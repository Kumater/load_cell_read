import spidev
import time

spi = spidev.SpiDev()
spi.open(0, 1)  # <-- changed to CE1
spi.max_speed_hz = 100000
spi.mode = 0

print("Trying spidev0.1 ...")
try:
    while True:
        raw = spi.xfer2([0x00] * 7)
        print([hex(x) for x in raw])
        time.sleep(0.05)
except KeyboardInterrupt:
    spi.close()