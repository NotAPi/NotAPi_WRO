#!/usr/bin/python

import time
import VL53L0X
import RPi.GPIO as GPIO
import csv
import datetime

from gpiozero import AngularServo

L_TOF_PIN = 22
R_TOF_PIN = 24
F_TOF_PIN = 25

IN1_PIN = 10 #motor pins
IN2_PIN = 11
ENA_PIN = 12

BUTTON_PIN = 16 #button pin

SERVO_PIN = 17 #servo pin

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

class VL53L0X_XSHUT(VL53L0X.VL53L0X):
    def __init__(self, xshut_pin, i2c_bus=1, i2c_address=0x29):
        super().__init__(i2c_bus=i2c_bus, i2c_address=i2c_address)
        if xshut_pin is None:
            raise ValueError("Pin number is required")
        self.xshut_pin = xshut_pin
        GPIO.setup(self.xshut_pin, GPIO.OUT)
        GPIO.output(self.xshut_pin, GPIO.LOW)

    def open(self):
        GPIO.output(self.xshut_pin, GPIO.HIGH)
        time.sleep(0.05) # TODO: verify
        super().open()

    def close(self):
        super().close()
        GPIO.output(self.xshut_pin, GPIO.LOW)

DIR_FORWARD = 1
DIR_STOPPED = 0
DIR_BACKWARD = -1

ANGLE_TURN = 50
ANGLE_MID = 50
ANGLE_LEFT = ANGLE_MID + ANGLE_TURN
ANGLE_RIGHT = ANGLE_MID - ANGLE_TURN

class Motor:
    def __init__(self, in1_pin, in2_pin, ena_pin, servo_pin):
        self.in1_pin = in1_pin
        self.in2_pin = in2_pin
        self.ena_pin = ena_pin
        self.servo_pin = servo_pin
        GPIO.setup(self.in1_pin, GPIO.OUT)
        GPIO.setup(self.ena_pin, GPIO.OUT)
        self.motor_pwm = GPIO.PWM(self.ena_pin, 420)
        self.motor_pwm.start(0)
        self.servo = AngularServo(pin=servo_pin)
        self.servo.angle = ANGLE_MID
        
    def set_direction(self, direction):
        GPIO.output(self.in1_pin, direction == DIR_BACKWARD)
        GPIO.output(self.in2_pin, direction == DIR_FORWARD)
    
    def __del__(self):
        self.set_direction(DIR_STOPPED)
        self.motor_pwm.stop()
        self.servo.angle = ANGLE_MID
    
    def forward(self, speed=100):
        self.set_direction(DIR_FORWARD)
        GPIO.output(self.ena_pin, GPIO.HIGH)
        self.motor_pwm.ChangeDutyCycle(speed)

    def backward(self, speed=100):
        self.set_direction(DIR_BACKWARD)
        GPIO.output(self.ena_pin, GPIO.HIGH)
        self.motor_pwm.ChangeDutyCycle(speed)

    def stop(self):
        self.set_direction(DIR_STOPPED)
        GPIO.output(self.ena_pin, GPIO.LOW)
        self.motor_pwm.ChangeDutyCycle(0)
        self.servo.angle = ANGLE_MID

    def giro_derecha(self):
        date_str = datetime.datetime.now().strftime("%Y-%m-%d")
        csv_filename = f"D_{date_str}.csv"

        with open(csv_filename, mode='a', newline='') as csvfile: 
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(["time", "distance_mm"])  

            self.stop()
            time.sleep(0.5)

            self.servo.angle = ANGLE_RIGHT
            self.forward()

            for _ in range(20):  # 2s / 0.1s
                current_time = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
                dist_l = tof_l.get_distance()
                csv_writer.writerow([current_time, dist_l])
                csvfile.flush()
                time.sleep(0.1)

            self.servo.angle = ANGLE_MID

    def giro_izquierda(self):
        date_str = datetime.datetime.now().strftime("%Y-%m-%d")
        csv_filename = f"I_{date_str}.csv"

        with open(csv_filename, mode='a', newline='') as csvfile: 
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(["time", "distance_mm"]) 
            self.stop()
            time.sleep(0.5)

            self.servo.angle = ANGLE_LEFT
            self.forward()

            for _ in range(20):  # 2s / 0.1s
                current_time = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
                dist_r = tof_r.get_distance()
                csv_writer.writerow([current_time, dist_r])
                csvfile.flush()
                time.sleep(0.1)

            self.servo.angle = ANGLE_MID

tof_l = VL53L0X_XSHUT(i2c_address=0x2a, xshut_pin=L_TOF_PIN)
tof_r = VL53L0X_XSHUT(i2c_address=0x2b, xshut_pin=R_TOF_PIN)
tof_f = VL53L0X_XSHUT(i2c_address=0x2c, xshut_pin=F_TOF_PIN)

tof_list = [
    tof_l,
    tof_r,
    tof_f,
]

for tof in tof_list:
    tof.open()
    tof.start_ranging(VL53L0X.Vl53l0xAccuracyMode.BETTER)

timing = tof_f.get_timing()
if timing < 20000:
    timing = 20000
print("Timing %d ms" % (timing/1000))

motor = Motor(IN1_PIN, IN2_PIN, ENA_PIN, SERVO_PIN)

print("ready")

try:
    while True:
        motor.servo.angle = ANGLE_MID
        motor.forward()
        dist_f = tof_f.get_distance()
        dist_l = tof_l.get_distance()
        dist_r = tof_r.get_distance()
        print(f"F: {dist_f}\tL: {dist_l}\tR: {dist_r}")
        if (dist_f < 200 and dist_f != 8191):
            if (dist_l > dist_r and dist_l != 8191):
                motor.giro_izquierda()
            elif (dist_r != 8191):
                motor.giro_derecha()
        time.sleep(0.05)

except KeyboardInterrupt:
    motor.stop()
    for tof in tof_list:
        tof.stop_ranging()
        tof.close()
    GPIO.cleanup()
