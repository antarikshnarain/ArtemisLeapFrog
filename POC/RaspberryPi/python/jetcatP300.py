#!/usr/bin/python3

import RPi.GPIO as gpio
import time
import serial

ctrl_pin = 25
pwr_pin = 11
commands = [
    "1,RTY,1",
    "1,RAC,1",
    "1,DHC,1",
    "1,RHC,1"
]

def readSerial(ser, delim):
    data = b''
    ch = ser.read()
    while ch != delim:
        data += ch
        ch = ser.read()
    return data.decode('utf-8')

gpio.setmode(gpio.BCM)
gpio.setup(ctrl_pin, gpio.OUT)
gpio.setup(pwr_pin, gpio.OUT)
gpio.output(ctrl_pin, gpio.LOW)
print("Powering up ECU")
gpio.output(pwr_pin, gpio.HIGH)
print("Opening Serial port")
ser = serial.Serial("/dev/ttyAMA3", 9600)
print("Sending commands")
for cmd in commands:
    ser.write(bytes(cmd+'\r', 'utf-8'))
    print("Written: ", cmd)
    time.sleep(0.01)
    if cmd == "1,DHC,1":
        print("Waiting for 10 sec.")
        time.sleep(10)
    print(readSerial(ser,b'\r'))
    
gpio.cleanup()