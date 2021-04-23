#!/usr/bin/python3

import RPi.GPIO as GPIO
import time

relay_pins = [18, 23, 24, 17, 27, 22, 16, 19]

GPIO.setmode(GPIO.BCM)
for pin in relay_pins:
    GPIO.setup(pin, GPIO.OUT)

# Play with relay
T = 2
while T:
    for pin in relay_pins:
        GPIO.output(pin, GPIO.HIGH)
        time.sleep(0.2)

    for pin in relay_pins:
        GPIO.output(pin, GPIO.LOW)
        time.sleep(0.2)
    
    T -= 1

GPIO.cleanup()
