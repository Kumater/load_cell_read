import spidev
import time

spi = spidev.SpiDev()
spi.open(0, 0)          # Bus 0, CE0
spi.max_speed_hz = 1000000  # 1 MHz — well within the 10 MHz max
spi.mode = 0b00         # SPI Mode 0

def read_encoder():
    # 50 bits needed; read 7 bytes (56 bits) to be safe
    raw = spi.xfer2([0x00] * 7)

    # Combine bytes into a single integer
    value = 0
    for byte in raw:
        value = (value << 8) | byte

    # Shift down to align — the 50 data bits start from MSB of transfer
    value >>= (56 - 50)

    # Extract fields
    mt = (value >> 19) & 0xFFFFFF   # bits 49:19 — 24 bits multi-turn
    st = (value >> 1)  & 0x3FFFF    # bits 18:1  — 18 bits single-turn
    n_err     = (value >> 0) & 0x1  # bit 0
    # nWarning would be next bit if you read 51 bits

    # Convert single-turn to degrees (18-bit = 262144 counts per rev)
    angle_deg = (st / 262144.0) * 360.0

    return mt, st, angle_deg, n_err

try:
    while True:
        mt, st, angle, err = read_encoder()
        print(f"Multi-turn: {mt:6d}  |  Raw ST: {st:6d}  |  Angle: {angle:8.3f}°  |  Error: {err}")
        time.sleep(0.1)
except KeyboardInterrupt:
    spi.close()