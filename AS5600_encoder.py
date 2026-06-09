import smbus2
import time

AS5600_ADDR = 0x36
bus_pitch = smbus2.SMBus(3)  # inner gimbal (θ)
bus_roll  = smbus2.SMBus(1)  # outer gimbal (φ)

PITCH_NEUTRAL = 311.84
ROLL_NEUTRAL  = 124.98

def read_angle(bus):
    data = bus.read_i2c_block_data(AS5600_ADDR, 0x0C, 2)
    raw = ((data[0] & 0x0F) << 8) | data[1]
    return raw * 360.0 / 4096.0

def to_signed(raw_deg, neutral_deg):
    delta = raw_deg - neutral_deg
    if delta > 180:
        delta -= 360
    elif delta <= -180:
        delta += 360
    return delta

def read_status(bus):
    status = bus.read_byte_data(AS5600_ADDR, 0x0B)
    md = (status >> 5) & 1
    return md

try:
    while True:
        md1 = read_status(bus_pitch)
        md2 = read_status(bus_roll)

        pitch = to_signed(read_angle(bus_pitch), PITCH_NEUTRAL) if md1 else None
        roll  = to_signed(read_angle(bus_roll),  ROLL_NEUTRAL)  if md2 else None

        pitch_str = f"{pitch:+.2f}°" if pitch is not None else "no magnet"
        roll_str  = f"{roll:+.2f}°"  if roll  is not None else "no magnet"

        print(f"Pitch: {pitch_str}   Roll: {roll_str}")
        time.sleep(0.05)

except KeyboardInterrupt:
    bus_pitch.close()
    bus_roll.close()