import sys
import time
import asyncio
import threading
import smbus2

from OptiTrack.NatNetClient import NatNetClient
import OptiTrack.MoCapData as MoCapData

from mavsdk import System
from mavsdk.mocap import (AngleBody, PositionBody, VisionPositionEstimate, Covariance)

import HelperFunctions as Helper

from pymavlink import mavutil

# ═══════════════════════════════════════════════════════════════════════════════
#  CONFIGURATION
# ═══════════════════════════════════════════════════════════════════════════════

serverAdress    = "192.168.0.100"
clientAdress    = "0.0.0.0"
assetID         = 21

# MAVSDK connects via mavlink-router UDP endpoint
px4Address      = "udp://:14540"

# pymavlink connects via a separate mavlink-router UDP endpoint
# See mavlink-router config at the bottom of this file.
mavlink_udp     = "udp:127.0.0.1:14541"

# Load cell (PGA302 on I2C bus 1)
I2C_BUS              = 1
PAGE_TEST_REG        = 0x40
PAGE_CTRL_STATUS_REG = 0x42
REG_MICRO_IF_CTRL    = 0x0C
REG_PADC_DATA_LSB    = 0x10
REG_PADC_DATA_MSB    = 0x11
REG_P_GAIN_SELECT    = 0x47
SOFT_RESET           = 0x02
ACCESS_DIGITAL_IF    = 0x01
GAIN_200             = 0x07

# DEBUG message index logged under debug_value in PX4 ULog (0–255, pick any unused)
DEBUG_INDEX          = 0

# Send rate
LOAD_CELL_SEND_INTERVAL = 0.05   # 20 Hz

# ═══════════════════════════════════════════════════════════════════════════════
#  GLOBALS
# ═══════════════════════════════════════════════════════════════════════════════

loop:          asyncio.AbstractEventLoop | None = None
natnet_client: NatNetClient | None              = None
natnet_thread: threading.Thread | None          = None
shutdown_event = threading.Event()
pose_queue:    asyncio.Queue | None             = None

latest_force: float = 0.0
force_lock = threading.Lock()


# ═══════════════════════════════════════════════════════════════════════════════
#  LOAD CELL  (PGA302 via I2C)  — logic unchanged from gimbal_sensor.py
# ═══════════════════════════════════════════════════════════════════════════════

def _lc_write(bus: smbus2.SMBus, page, reg, value):
    bus.write_byte_data(page, reg, value)

def _lc_read_padc(bus: smbus2.SMBus) -> float:
    lsb = bus.read_byte_data(PAGE_TEST_REG, REG_PADC_DATA_LSB)
    msb = bus.read_byte_data(PAGE_TEST_REG, REG_PADC_DATA_MSB)
    combined = (msb << 8) | lsb
    if combined > 32767:
        combined -= 65536
    return float(combined)

def _lc_setup(bus: smbus2.SMBus):
    print("[LoadCell] Initializing PGA302...")
    _lc_write(bus, PAGE_TEST_REG,        REG_MICRO_IF_CTRL, SOFT_RESET)
    time.sleep(0.1)
    _lc_write(bus, PAGE_TEST_REG,        REG_MICRO_IF_CTRL, ACCESS_DIGITAL_IF)
    time.sleep(0.1)
    _lc_write(bus, PAGE_CTRL_STATUS_REG, REG_P_GAIN_SELECT, GAIN_200)
    time.sleep(0.1)
    print("[LoadCell] PGA302 ready.")

def load_cell_worker():
    """Reads the PGA302 at 20 Hz and keeps latest_force up to date."""
    global latest_force
    try:
        bus = smbus2.SMBus(I2C_BUS)
        _lc_setup(bus)
        while not shutdown_event.is_set():
            val = _lc_read_padc(bus)
            with force_lock:
                latest_force = val
            time.sleep(LOAD_CELL_SEND_INTERVAL)
    except Exception as e:
        print(f"[LoadCell] Worker error: {e}")
    print("[LoadCell] Thread stopped.")


# ═══════════════════════════════════════════════════════════════════════════════
#  MAVLink DEBUG sender
#
#  Uses MAVLink message DEBUG (id 251) which maps to the uORB topic
#  `debug_value` and IS recorded in the PX4 ULog by default.
#
#  In Flight Review / PlotJuggler look for:
#      debug_value  →  value   (field contains the raw load cell reading)
#      debug_value  →  ind     (will equal DEBUG_INDEX = 0)
#
#  pymavlink connects to mavlink-router's second UDP port so it never
#  touches /dev/ttyAMA0 directly — that port is owned by MAVSDK alone.
# ═══════════════════════════════════════════════════════════════════════════════

def mavlink_debug_worker():
    try:
        mav = mavutil.mavlink_connection(
            mavlink_udp,
            source_system=1,
            source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER,
        )
        print("[MAVLink] DEBUG sender connected via UDP.")
        boot_ms = int(time.monotonic() * 1000)

        while not shutdown_event.is_set():
            with force_lock:
                force = latest_force

            time_boot_ms = int(time.monotonic() * 1000) - boot_ms

            # MAVLink DEBUG message — always logged by PX4 as debug_value
            mav.mav.debug_send(
                time_boot_ms,   # time_boot_ms  (uint32)
                DEBUG_INDEX,    # ind           (uint8)  — identifies this channel
                float(force),   # value         (float)
            )
            time.sleep(LOAD_CELL_SEND_INTERVAL)

    except Exception as e:
        print(f"[MAVLink] DEBUG sender error: {e}")
    print("[MAVLink] DEBUG sender stopped.")


# ═══════════════════════════════════════════════════════════════════════════════
#  OPTITRACK  — callback and worker UNCHANGED from RPi_OptiTrack.py
# ═══════════════════════════════════════════════════════════════════════════════

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

        data = (curr_Pos, curr_Ang, time_usec)

        def put_latest():
            try:
                pose_queue.get_nowait()
            except asyncio.QueueEmpty:
                pass
            try:
                pose_queue.put_nowait(data)
            except asyncio.QueueFull:
                pass

        loop.call_soon_threadsafe(put_latest)

    except Exception as e:
        print("Callback Error:", e)


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


# ═══════════════════════════════════════════════════════════════════════════════
#  MAIN ASYNC ENTRY
# ═══════════════════════════════════════════════════════════════════════════════

async def configDroneAndRun():
    global loop, pose_queue
    loop = asyncio.get_running_loop()
    pose_queue = asyncio.Queue(maxsize=1)

    # 1. Connect to PX4 via MAVSDK (through mavlink-router UDP)
    drone = System()
    print("Connecting to PX4...")
    await drone.connect(system_address=px4Address)

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected to drone!")
            break

    # 2. Start OptiTrack thread (same order as original)
    global natnet_thread
    natnet_thread = threading.Thread(target=natnet_worker, daemon=True)
    natnet_thread.start()

    # 3. Start load cell reader thread
    lc_thread = threading.Thread(target=load_cell_worker, daemon=True)
    lc_thread.start()

    # 4. Start MAVLink DEBUG sender thread
    mav_debug_thread = threading.Thread(target=mavlink_debug_worker, daemon=True)
    mav_debug_thread.start()

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
                with force_lock:
                    force_snapshot = latest_force
                print(
                    f"x={curr_Pos.x_m:.2f} y={curr_Pos.y_m:.2f} z={curr_Pos.z_m:.2f} "
                    f"roll={curr_Ang.roll_rad:.2f} pitch={curr_Ang.pitch_rad:.2f} yaw={curr_Ang.yaw_rad:.2f} "
                    f"| force={force_snapshot:.1f} raw"
                )
                last_print = now
    except asyncio.CancelledError:
        pass


# ─── runExternal kept for compatibility ───────────────────────────────────────

async def runExternal(drone, ext_loop):
    global loop, pose_queue
    loop = ext_loop
    pose_queue = asyncio.Queue(maxsize=1)

    global natnet_thread
    natnet_thread = threading.Thread(target=natnet_worker, daemon=True)
    natnet_thread.start()

    lc_thread = threading.Thread(target=load_cell_worker, daemon=True)
    lc_thread.start()

    mav_debug_thread = threading.Thread(target=mavlink_debug_worker, daemon=True)
    mav_debug_thread.start()

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
                with force_lock:
                    force_snapshot = latest_force
                print(
                    f"x={curr_Pos.x_m:.2f} y={curr_Pos.y_m:.2f} z={curr_Pos.z_m:.2f} "
                    f"roll={curr_Ang.roll_rad:.2f} pitch={curr_Ang.pitch_rad:.2f} yaw={curr_Ang.yaw_rad:.2f} "
                    f"| force={force_snapshot:.1f} raw"
                )
                last_print = now
    except asyncio.CancelledError:
        pass


# ─── Graceful shutdown ────────────────────────────────────────────────────────

def shutdown():
    print("\n[Main] Shutting down...")
    shutdown_event.set()
    if natnet_thread:
        natnet_thread.join(timeout=2)
    print("[Main] Done.")


##### Entry #####
if __name__ == "__main__":
    print("RPi_OptiTrack_LoadCell Started")
    try:
        asyncio.run(configDroneAndRun())
    except KeyboardInterrupt:
        shutdown()
