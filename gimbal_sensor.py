import smbus2
import time

# Memory Page Addresses (Slave Addresses)[cite: 2]
PAGE_TEST_REG          = 0x40
PAGE_CTRL_STATUS_REG   = 0x42

# Registers[cite: 2]
REG_MICRO_IF_CTRL      = 0x0C
REG_PADC_DATA_LSB      = 0x10
REG_PADC_DATA_MSB      = 0x11
REG_P_GAIN_SELECT      = 0x47

# Settings[cite: 2]
SOFT_RESET             = 0x02
ACCESS_DIGITAL_IF      = 0x01
GAIN_200               = 0x07

# Initialize I2C bus 1
bus = smbus2.SMBus(1)

def write_register(page, reg, value):
    """Writes a value to a specific page and register[cite: 2]."""
    bus.write_byte_data(page, reg, value)

def read_padc():
    """Reads and combines the 16-bit PADC data[cite: 2]."""
    # Read LSB and MSB from Page 0x40[cite: 2]
    lsb = bus.read_byte_data(PAGE_TEST_REG, REG_PADC_DATA_LSB)
    msb = bus.read_byte_data(PAGE_TEST_REG, REG_PADC_DATA_MSB)
    
    # Combine bytes into a 16-bit signed integer[cite: 2]
    combined = (msb << 8) | lsb
    
    # Handle two's complement for signed 16-bit[cite: 2]
    if combined > 32767:
        combined -= 65536
    return combined

def setup_pga302():
    print("Initializing PGA302...")
    
    # 1. Soft Reset (Targeting Page 0x40)[cite: 2]
    write_register(PAGE_TEST_REG, REG_MICRO_IF_CTRL, SOFT_RESET)
    time.sleep(0.1)
    
    # 2. Access Digital Interface (Targeting Page 0x40)[cite: 2]
    write_register(PAGE_TEST_REG, REG_MICRO_IF_CTRL, ACCESS_DIGITAL_IF)
    time.sleep(0.1)
    
    # 3. Set Gain to 200x (Targeting Page 0x42)[cite: 2]
    write_register(PAGE_CTRL_STATUS_REG, REG_P_GAIN_SELECT, GAIN_200)
    time.sleep(0.1)
    
    print("Initialization Complete. Streaming Raw Data:")

if __name__ == "__main__":
    try:
        setup_pga302()
        while True:
            raw_val = read_padc()
            print(f"Raw Value: {raw_val}")
            time.sleep(0.1) # 100ms delay as in loop()[cite: 2]
    except KeyboardInterrupt:
        print("\nStream stopped by user.")
