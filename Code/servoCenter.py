#!/usr/bin/python

import csv
import datetime
import os
import pigpio
import time
import VL53L0X

import RPi.GPIO as GPIO

from gpiozero import AngularServo

try:
    os.system("sudo pigpiod")  # Launching GPIO library
    time.sleep(1)  # As it takes some time to launch
except:
    #aaaprint("GPIO library already launched")
    pass

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)  

ANGLE_MID = 0
ANGLE_LEFT = ANGLE_MID + 45
ANGLE_RIGHT = ANGLE_MID - 60

pi = pigpio.pi()

if False:
    servo = AngularServo(pin=17)
    #servo.angle = -50
    servo.angle = ANGLE_MID
    time.sleep(2)
    servo.angle = ANGLE_LEFT
    time.sleep(2)
    servo.angle = ANGLE_RIGHT
    time.sleep(2)

def do_servo(angle=90):
    pulse_width = 500 + (angle / 180.0) * 2000
    pi.set_servo_pulsewidth(17, pulse_width)

# for i in range(0, 180):
#     do_servo(i)
#     print(i)

do_servo(90)