import spidev
import time

# Initialize SPI
spi = spidev.SpiDev()
spi.open(0, 0) # Open bus 0, device 0 (CE0)

# AS20 supports up to 10Mbps [cite: 21, 450]
spi.max_speed_hz = 5000000 
spi.mode = 0b00 # Standard SPI Mode 0

def read_encoder():
    # The AS20 returns 24 bits of MT data + 18 bits of ST data [cite: 369, 370]
    # We read 6 bytes (48 bits) to cover the full 42-bit resolution [cite: 522]
    try:
        # Request data (sending dummy bytes to pulse the clock)
        resp = spi.xfer2([0x00] * 6)
        
        # Combine bytes into a single integer
        # Note: Alignment may vary based on start-up; check documentation for MSB 
        full_data = (resp[0] << 40) | (resp[1] << 32) | (resp[2] << 24) | \
                    (resp[3] << 16) | (resp[4] << 8) | resp[5]
        
        # Masking according to datasheet (42 bits total) [cite: 518, 522]
        mt_data = (full_data >> 20) & 0xFFFFFF  # 24-bit Multi-Turn [cite: 369]
        st_data = (full_data >> 2) & 0x3FFFF    # 18-bit Single-Turn [cite: 370]
        
        print(f"Multi-Turn: {mt_data} | Single-Turn: {st_data}")
        
    except Exception as e:
        print(f"Error reading encoder: {e}")

try:
    while True:
        read_encoder()
        time.sleep(0.1) # 10Hz update rate
except KeyboardInterrupt:
    spi.close()