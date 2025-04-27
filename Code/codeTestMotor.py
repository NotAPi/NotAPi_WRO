#!/usr/bin/env python3

import numpy as np
import time
import pigpio
import os

#chupapitos


#se conectó bien: aiuda

try:
    os.system("sudo pigpiod")  # Launching GPIO library
    time.sleep(1)  # As it takes some time to launch
except:
    #aaaprint("GPIO library already launched")
    pass


# Initialize the Raspberry Pi camera


L_TOF_PIN = 22
R_TOF_PIN = 24
F_TOF_PIN = 25

IN1_PIN = 10 #motor pins
IN2_PIN = 11
ENA_PIN = 12

BUTTON_PIN = 16 #button pin

SERVO_PIN = 17 #servo pin
angulo_objetivo = 105 #lo pone en el centro 

# Initialize pigpio
pi = pigpio.pi()

# Allow the camera to warm up
time.sleep(0.1)


def forward(speed=255):
    pi.write(IN1_PIN, 0)
    pi.write(IN2_PIN, 1)
    pi.set_PWM_dutycycle(ENA_PIN, speed)

def backward(speed=255):
    pi.write(IN1_PIN, 1)
    pi.write(IN2_PIN, 0)
    pi.set_PWM_dutycycle(ENA_PIN, speed)

def stop():
    pi.write(IN1_PIN, 0)
    pi.write(IN2_PIN, 0)
    pi.set_PWM_dutycycle(ENA_PIN, 0)
    servo(105)

def servo(angle):
    pulse_width = 500 + (angle / 180.0) * 2000
    pi.set_servo_pulsewidth(SERVO_PIN, pulse_width)

def giro_derecha():
    stop()
    time.sleep(0.5)
    servo(105)
    backward()
    time.sleep(2)
    servo(55)
    backward(255)
    time.sleep(2)
    servo(105)
    time.sleep(1)

def giro_izquierda():
    stop()
    time.sleep(0.5)
    servo(105)
    backward()
    time.sleep(2)
    servo(155)
    backward(255)
    time.sleep(2)
    servo(105)
    time.sleep(1)
        
stop()

print("ready")


try:
    
    while True:
        

            forward()
            servo(105)
        


except KeyboardInterrupt:
    stop()

    pi.stop()
