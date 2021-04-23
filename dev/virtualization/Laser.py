import serial
import sys
import time

class RFD900X
    
    def __init__(self, port, baud):
        self.rf_serial = serial.Serial(port, baudrate = baud,
            parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=1)
        self.values = ["0x59","0x59","0x10","0x02","0xff","0x03","0x02","0x00","0xc8"]
    def simulate(self):
        while 1:
            self.rf_serial.write("".join('%c'%int(v,0) for v in a))
            time.sleep(0.1)

if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise ValueError("Pass port and baud as parameter.")
    rfd = RFD900X(sys.argv[1], int(sys.argv[2]))
    rfd.simulate()