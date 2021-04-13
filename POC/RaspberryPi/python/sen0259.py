"""
Laser Sensor Read
"""
import serial
import time

ser = serial.Serial("/dev/ttyAMA1", 115200)

while 1:
    #data = ser.readline()
    # Read given number of bytes
    data = ser.read(size=9)

    # If data is string format convert pair wise to hex
    print (len(data))
    if len(data) == 9:
        header = int(data[0]) + (int(data[1]) << 8)
        dist = int(data[2]) + (int(data[3]) << 8)
        signal = int(data[4]) + (int(data[5]) << 8)
        mode = int(data[6])
        spare = int(data[7])
        check_sum = int(data[8])
        print(header, dist, signal, mode, spare, check_sum)
    #print(data)
    time.sleep(0.1)
