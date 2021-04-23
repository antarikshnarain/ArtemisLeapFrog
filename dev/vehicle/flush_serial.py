#!/usr/bin/python3
import serial
import RPi.GPIO as GPIO
import time

ports = [("/dev/ttyAMA2",115200), ("/dev/ttyAMA3",9600), ("/dev/ttyAMA4",57600)]

for port,baud in ports:
    print("Cleaning: ", port, " at ", baud)
    ser = serial.Serial(port, baud)
    ser.flushInput()
    ser.flushOutput()
    ser.close()

relay_pins = [25, 11, 18, 23, 24, 17, 27, 22, 16, 19]
GPIO.setmode(GPIO.BCM)
GPIO.setup(25, GPIO.OUT)
GPIO.setup(11, GPIO.OUT)
GPIO.output(25, GPIO.LOW)
GPIO.output(11, GPIO.HIGH)

# Do not do cleanup to maintain the gpio state