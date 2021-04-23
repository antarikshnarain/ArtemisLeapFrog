import serial
import time
import logging

ser = serial.Serial("/dev/pts/7", 57600)
logging.basicConfig(filename="serial.log", level=logging.DEBUG)

while 1:
    data = ser.read_all()
    if len(data) > 0:
        print("Received data: ", data)
        logging.info(data)
        if data == b"exit\x03":
            break
        ser.write(data)
    time.sleep(1)

print("Exiting code!")
    