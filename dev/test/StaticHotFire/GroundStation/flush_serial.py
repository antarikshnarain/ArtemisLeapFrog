#!/usr/bin/python3
import serial

ports = [("/dev/ttyACM0",57600)]

for port,baud in ports:
    print("Cleaning: ", port, " at ", baud)
    ser = serial.Serial(port, baud)
    ser.flushInput()
    ser.flushOutput()
    ser.close()