import smbus2
import time

AS5600_ADDR    = 0x36
PAGE_TEST_REG  = 0x40
PAGE_CTRL_REG  = 0x42

REG_MICRO_IF_CTRL = 0x0C
REG_PADC_DATA_LSB = 0x10
REG_PADC_DATA_MSB = 0x11
REG_P_GAIN_SELECT = 0x47

SOFT_RESET        = 0x02
ACCESS_DIGITAL_IF = 0x01
GAIN_200          = 0x07

PITCH_NEUTRAL = 311.84
ROLL_NEUTRAL  = 124.98

bus_pitch = smbus2.SMBus(3)  # inner gimbal (θ)
bus_roll  = smbus2.SMBus(1)  # outer gimbal (φ)
bus_load  = smbus2.SMBus(4)

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
    return (status >> 5) & 1

def setup_pga302():
    bus_load.write_byte_data(PAGE_TEST_REG, REG_MICRO_IF_CTRL, SOFT_RESET)
    time.sleep(0.1)
    bus_load.write_byte_data(PAGE_TEST_REG, REG_MICRO_IF_CTRL, ACCESS_DIGITAL_IF)
    time.sleep(0.1)
    bus_load.write_byte_data(PAGE_CTRL_REG, REG_P_GAIN_SELECT, GAIN_200)
    time.sleep(0.1)
    print("PGA302 initialized")

def read_padc():
    lsb = bus_load.read_byte_data(PAGE_TEST_REG, REG_PADC_DATA_LSB)
    msb = bus_load.read_byte_data(PAGE_TEST_REG, REG_PADC_DATA_MSB)
    combined = (msb << 8) | lsb
    if combined > 32767:
        combined -= 65536
    return combined

try:
    setup_pga302()
    while True:
        md1 = read_status(bus_pitch)
        md2 = read_status(bus_roll)

        pitch = to_signed(read_angle(bus_pitch), PITCH_NEUTRAL) if md1 else None
        roll  = to_signed(read_angle(bus_roll),  ROLL_NEUTRAL)  if md2 else None
        load  = read_padc()

        pitch_str = f"{pitch:+.2f}°" if pitch is not None else "no magnet"
        roll_str  = f"{roll:+.2f}°"  if roll  is not None else "no magnet"

        print(f"Pitch: {pitch_str}   Roll: {roll_str}   Load: {load}")
        time.sleep(0.05)

except KeyboardInterrupt:
    bus_pitch.close()
    bus_roll.close()
    bus_load.close()