import sys
import time
import asyncio
import threading
import signal

from OptiTrack.NatNetClient import NatNetClient
import OptiTrack.MoCapData as MoCapData 

from mavsdk import System
from mavsdk.mocap import (AngleBody, PositionBody, VisionPositionEstimate, Covariance)

import HelperFunctions as Helper

##### Configuration #####
serverAdress = "192.168.0.100"
clientAdress = "0.0.0.0"
assetID = 21
px4Address = "serial:///dev/ttyAMA0:500000"
##### Configuration #####

loop: asyncio.AbstractEventLoop | None = None
natnet_client: NatNetClient | None = None
natnet_thread: threading.Thread | None = None
shutdown_event = threading.Event()

##### Latest-only pose queue #####
pose_queue: asyncio.Queue | None = None

##### Callback #####
def receive_rigid_body_frame(rb_id, position, orientation):
    if rb_id != assetID:
        return

    global pose_queue, loop
    if pose_queue is None or loop is None:
        return

    try:
        time_usec = int(time.time() * 1_000_000)
        curr_Pos = PositionBody(position[0], -position[1], -position[2])
        eulAng = Helper.euler_from_quaternion(
            orientation[0], orientation[2], -orientation[1], orientation[3]
        )
        curr_Ang = AngleBody(eulAng[0], eulAng[1], eulAng[2])

        # Prepare tuple
        data = (curr_Pos, curr_Ang, time_usec)

        # Thread-safe enqueue: drop old item first
        def put_latest():
            try:
                pose_queue.get_nowait()   # drop old
            except asyncio.QueueEmpty:
                pass
            try:
                pose_queue.put_nowait(data)
            except asyncio.QueueFull:
                pass

        loop.call_soon_threadsafe(put_latest)

    except Exception as e:
        print("Callback Error:", e)


##### OptiTrack thread #####
def natnet_worker():
    global natnet_client
    try:
        client = NatNetClient()
        natnet_client = client
        client.set_client_address(clientAdress)
        client.set_server_address(serverAdress)
        client.set_use_multicast(True)
        client.set_print_level(0)
        client.rigid_body_listener = receive_rigid_body_frame

        print("[OptiTrack] Starting streaming client...")
        client.run('d')
        time.sleep(1)
        if not client.connected():
            print("[OptiTrack] ERROR: Connection failed. Check Motive streaming settings.")
            return
        Helper.print_configuration(client)
        print("[OptiTrack] Streaming active.")
        while not shutdown_event.is_set():
            time.sleep(0.05)
    except Exception as e:
        print(f"[OptiTrack] Worker error: {e}")
    finally:
        if natnet_client:
            try:
                natnet_client.shutdown()
            except Exception:
                pass
        print("[OptiTrack] Thread stopped.")




async def configDroneAndRun():
    global loop, pose_queue
    loop = asyncio.get_running_loop()
    pose_queue = asyncio.Queue(maxsize=1)

    # 1. CONNECT FIRST
    drone = System()
    print("Connecting to PX4...")
    await drone.connect(system_address=px4Address)

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected to drone!")
            break

    # 2. START OPTITRACK AFTER CONNECTION
    global natnet_thread
    natnet_thread = threading.Thread(target=natnet_worker, daemon=True)
    natnet_thread.start()
    
    # ... rest of the code


    last_print = time.time()
    try:
        while True:
            curr_Pos, curr_Ang, time_usec = await pose_queue.get()
            vis_pos_est = VisionPositionEstimate(
                time_usec, curr_Pos, curr_Ang, Covariance([float('nan')])
            )
            await drone.mocap.set_vision_position_estimate(vis_pos_est)

            now = time.time()
            if now - last_print >= 1:
                print(
                    f"x={curr_Pos.x_m:.2f} y={curr_Pos.y_m:.2f} z={curr_Pos.z_m:.2f} "
                    f"roll={curr_Ang.roll_rad:.2f} pitch={curr_Ang.pitch_rad:.2f} yaw={curr_Ang.yaw_rad:.2f}"
                )
                last_print = now
    except asyncio.CancelledError:
        pass


##### External loop version #####
async def runExternal(drone, ext_loop):
    global loop, pose_queue
    loop = ext_loop
    pose_queue = asyncio.Queue(maxsize=1)

    global natnet_thread
    natnet_thread = threading.Thread(target=natnet_worker, daemon=True)
    natnet_thread.start()

    await asyncio.sleep(5)

    print("Waiting for first pose data...")
    try:
        await asyncio.wait_for(pose_queue.get(), timeout=5.0)
        print("Received first pose data.")
    except asyncio.TimeoutError:
        print("No pose data received after 5s. Continuing anyway...")

    last_print = time.time()
    try:
        while True:
            curr_Pos, curr_Ang, time_usec = await pose_queue.get()
            vis_pos_est = VisionPositionEstimate(
                time_usec, curr_Pos, curr_Ang, Covariance([float('nan')])
            )
            await drone.mocap.set_vision_position_estimate(vis_pos_est)

            now = time.time()
            if now - last_print >= 1:
                print(
                    f"x={curr_Pos.x_m:.2f} y={curr_Pos.y_m:.2f} z={curr_Pos.z_m:.2f} "
                    f"roll={curr_Ang.roll_rad:.2f} pitch={curr_Ang.pitch_rad:.2f} yaw={curr_Ang.yaw_rad:.2f}"
                )
                last_print = now
    except asyncio.CancelledError:
        pass


##### Graceful shutdown #####
def shutdown():
    print("\n[Main] Shutting down...")
    shutdown_event.set()
    if natnet_thread:
        natnet_thread.join(timeout=2)
    print("[Main] Done.")


##### Entry #####
if __name__ == "__main__":
    print("RPi4_OptiTrack Started")
    try:
        asyncio.run(configDroneAndRun())
    except KeyboardInterrupt:
        shutdown()
