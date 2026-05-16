import spidev
import time

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 100000  # slow so multimeter can catch it
spi.mode = 0

print("Starting continuous read loop - probe pins now...")
print("Press Ctrl+C to stop")
print("-" * 50)

try:
    while True:
        raw = spi.xfer2([0x00] * 7)
        print([hex(x) for x in raw])
        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nStopped.")
    spi.close()