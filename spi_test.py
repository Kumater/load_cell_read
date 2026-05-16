import spidev
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 100000
spi.mode = 0

result = spi.xfer2([0xAA, 0x55, 0xFF])
print([hex(x) for x in result])
# Should print: ['0xaa', '0x55', '0xff']
# If you get all 0x00, SPI itself is broken