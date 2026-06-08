import smbus2
import time

AS5600_ADDR = 0x36
bus_pitch = smbus2.SMBus(1)  # inner gimbal (θ)
bus_roll  = smbus2.SMBus(3)  # outer gimbal (φ)

def read_angle(bus):
    data = bus.read_i2c_block_data(AS5600_ADDR, 0x0C, 2)
    raw = ((data[0] & 0x0F) << 8) | data[1]
    return raw * 360.0 / 4096.0

def read_status(bus):
    status = bus.read_byte_data(AS5600_ADDR, 0x0B)
    md = (status >> 5) & 1
    ml = (status >> 4) & 1
    mh = (status >> 3) & 1
    return md, ml, mh

try:
    while True:
        md1, ml1, mh1 = read_status(bus_pitch)
        md2, ml2, mh2 = read_status(bus_roll)

        pitch = read_angle(bus_pitch) if md1 else None
        roll  = read_angle(bus_roll)  if md2 else None

        pitch_str = f"{pitch:.2f}°" if pitch is not None else "no magnet"
        roll_str  = f"{roll:.2f}°"  if roll  is not None else "no magnet"

        print(f"Pitch: {pitch_str}   Roll: {roll_str}")
        time.sleep(0.05)

except KeyboardInterrupt:
    bus_pitch.close()
    bus_roll.close()