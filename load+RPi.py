import sys
import time
import asyncio
import threading
import signal
import smbus2 # Required for I2C communication[cite: 2]

from OptiTrack.NatNetClient import NatNetClient
import OptiTrack.MoCapData as MoCapData 

from mavsdk import System
from mavsdk.mocap import (AngleBody, PositionBody, VisionPositionEstimate, Covariance)
from mavsdk.telemetry import DebugVect # Required for logging force

import HelperFunctions as Helper

##### Configuration #####
serverAdress = "192.168.0.100"
clientAdress = "0.0.0.0"
assetID = 18
px4Address = "serial:///dev/ttyAMA0:500000"

# PGA302 I2C Addresses[cite: 1, 2]
PAGE_TEST_REG = 0x40
REG_PADC_DATA_LSB = 0x10
REG_PADC_DATA_MSB = 0x11

# Calibrated Constants[cite: 2]
OFFSET = 93.5
SCALE_FACTOR = 238.66 # raw units per kg
##### Configuration #####

bus = smbus2.SMBus(1)

def get_weight_kg():
    """Reads from PGA302 and returns weight in kg[cite: 1, 2]."""
    try:
        lsb = bus.read_byte_data(PAGE_TEST_REG, REG_PADC_DATA_LSB)
        msb = bus.read_byte_data(PAGE_TEST_REG, REG_PADC_DATA_MSB)
        combined = (msb << 8) | lsb
        if combined > 32767:
            combined -= 65536
        
        # Apply calibration formula: (Raw - Offset) / Scale
        weight_kg = (combined - OFFSET) / SCALE_FACTOR
        return float(weight_kg)
    except Exception as e:
        return 0.0

# ... (Keep your existing receive_rigid_body_frame and natnet_worker) ...

##### Drone loop #####
async def configDroneAndRun():
    global loop, pose_queue
    loop = asyncio.get_running_loop()
    pose_queue = asyncio.Queue(maxsize=1)

    global natnet_thread
    natnet_thread = threading.Thread(target=natnet_worker, daemon=True)
    natnet_thread.start()

    await asyncio.sleep(5)

    drone = System()
    print("Connecting to PX4...")
    await drone.connect(system_address=px4Address)

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected to drone!")
            break

    last_print = time.time()
    try:
        while True:
            # 1. Get latest OptiTrack pose from queue
            curr_Pos, curr_Ang, time_usec = await pose_queue.get()
            
            # 2. Sample the Load Cell[cite: 1, 2]
            force_kg = get_weight_kg()

            # 3. Send Vision Position to PX4 (EKE/Navigation)
            vis_pos_est = VisionPositionEstimate(
                time_usec, curr_Pos, curr_Ang, Covariance([float('nan')])
            )
            await drone.mocap.set_vision_position_estimate(vis_pos_est)

            # 4. Send Force to PX4 for Logging
            # We map force to the 'x' component of a DebugVect named "FORCE"
            log_data = DebugVect("FORCE", time_usec, force_kg, 0.0, 0.0)
            await drone.telemetry.send_debug_vect(log_data)

            now = time.time()
            if now - last_print >= 1:
                print(
                    f"Pos: [{curr_Pos.x_m:.2f}, {curr_Pos.y_m:.2f}, {curr_Pos.z_m:.2f}] "
                    f"Force: {force_kg:.3f} kg"
                )
                last_print = now
    except asyncio.CancelledError:
        pass

# ... (Rest of the script remains same) ...