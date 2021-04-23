from smbus import SMBus
import time


if __name__ == "__main__":
    i2cbus = SMBus(1)
    i2caddress = 0x68
    i2cbus.write_byte_data(i2caddress, IOCON, 0x02)
    