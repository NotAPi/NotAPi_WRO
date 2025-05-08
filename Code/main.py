#!/usr/bin/python

# cSpell:disable

import csv
import datetime
import os
import pigpio
import statistics
import time
import VL53L0X

import RPi.GPIO as GPIO

from statemachine import StateMachine, State

try:
    os.system("sudo pigpiod")  # Launching GPIO library
    time.sleep(1)  # As it takes some time to launch
except:
    pass

pi = pigpio.pi()

DO_CSV = not True

#
# ToF sensor accuracy modes (as per library)
#
# - GOOD        # 33 ms timing budget 1.2m range
# - BETTER      # 66 ms timing budget 1.2m range
# - BEST        # 200 ms 1.2m range
# - LONG_RANGE  # 33 ms timing budget 2m range
# - HIGH_SPEED  # 20 ms timing budget 1.2m range
#
#ACCURACY_MODE = VL53L0X.Vl53l0xAccuracyMode.BETTER
ACCURACY_MODE = VL53L0X.Vl53l0xAccuracyMode.LONG_RANGE

# ToF sensors
L_TOF_PIN = 22
R_TOF_PIN = 24
F_TOF_PIN = 25

# Motor driver
IN1_PIN = 10
IN2_PIN = 11
ENA_PIN = 12

# Button? pin
BUTTON_PIN = 16

# Servo
SERVO_PIN = 17

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

SENSOR_HIST_LENGTH = 5

class VL53L0X_XSHUT(VL53L0X.VL53L0X):
    def __init__(self, xshut_pin, i2c_bus=1, i2c_address=0x29):
        super().__init__(i2c_bus=i2c_bus, i2c_address=i2c_address)
        if xshut_pin is None:
            raise ValueError("Pin number is required")
        self.xshut_pin = xshut_pin
        GPIO.setup(self.xshut_pin, GPIO.OUT)
        GPIO.output(self.xshut_pin, GPIO.LOW)
        self.history = []

    def open(self):
        GPIO.output(self.xshut_pin, GPIO.HIGH)
        time.sleep(0.05) # Very important delay, do not remove
        super().open()

    def close(self):
        super().close()
        GPIO.output(self.xshut_pin, GPIO.LOW)
        
    def get_distance(self):
        #return super().get_distance()
        if len(self.history) >= SENSOR_HIST_LENGTH:
            self.history.pop(0)
        self.history.append(super().get_distance())
        return int(statistics.median(self.history))
        #return statistics.fmean(self.history)
    
    def get_raw_distance(self):
        if len(self.history) == 0:
            return self.get_distance()
        else:
            return self.history[-1]

DIR_FORWARD = 1
DIR_STOPPED = 0
DIR_BACKWARD = -1

ANGLE_TURN = 45

class Servo:
    def __init__(self, pin, sweep_angle, offset=0):
        self.pin = pin
        self.sweep_angle = sweep_angle
        self.offset = offset
        self.mid()

    def set_angle(self, angle=0):
        # todo: I wanted to write a comment in here
        #       but I forgot what I wanted to write
        pulse_width = 500 + ((angle + self.offset + 90) / 180.0) * 2000
        pi.set_servo_pulsewidth(self.pin, pulse_width)

    def mid(self):
        self.set_angle()

    def left(self):
        self.set_angle(-self.sweep_angle)

    def right(self):
        self.set_angle(self.sweep_angle)

class Motor:
    def __init__(self, in1_pin, in2_pin, ena_pin):
        self.in1_pin = in1_pin
        self.in2_pin = in2_pin
        self.ena_pin = ena_pin
        GPIO.setup(self.in1_pin, GPIO.OUT)
        GPIO.setup(self.in2_pin, GPIO.OUT)
        GPIO.setup(self.ena_pin, GPIO.OUT)
        self.motor_pwm = GPIO.PWM(self.ena_pin, 420) # PWM frequency
        self.motor_pwm.start(0)

    def set_direction(self, direction):
        GPIO.output(self.in1_pin, direction == DIR_BACKWARD)
        GPIO.output(self.in2_pin, direction == DIR_FORWARD)

    def __del__(self):
        # Re-do GPIO setup to prevent errors when stopping the program
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.in1_pin, GPIO.OUT)
        GPIO.setup(self.in2_pin, GPIO.OUT)
        GPIO.setup(self.ena_pin, GPIO.OUT)
        self.set_direction(DIR_STOPPED)
        self.motor_pwm.stop()

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

motor = Motor(IN1_PIN, IN2_PIN, ENA_PIN)
servo = Servo(pin=SERVO_PIN, sweep_angle=ANGLE_TURN, offset=-4.615)

tof_f = VL53L0X_XSHUT(i2c_address=0x2a, xshut_pin=F_TOF_PIN)
tof_l = VL53L0X_XSHUT(i2c_address=0x2b, xshut_pin=L_TOF_PIN)
tof_r = VL53L0X_XSHUT(i2c_address=0x2c, xshut_pin=R_TOF_PIN)

tof_list = [
    tof_f,
    tof_l,
    tof_r,
]

for tof in tof_list:
    tof.open()
    tof.start_ranging(ACCURACY_MODE)

timing = tof_f.get_timing()
if timing < 20000:
    timing = 20000
print("Timing %d ms" % (timing / 1000))

print("ready")

MIN_F_DIST = 450
MIN_LR_DIST = 125
NUM_CYCLES_F = 0
NUM_CYCLES_LR = 0

try:
    class CarStateMachine(StateMachine):
        forward = State(initial=True)
        left = State()
        right = State()
        stopped = State(final=True)

        turn_left = forward.to(left)
        turn_right = forward.to(right)
        go_forward = (left.to(forward) | right.to(forward))
        stop_car = (forward.to(stopped) | left.to(stopped) | right.to(stopped))

        def before_stop_car(self, event: str, source: State, target: State, message: str = ""):
            message = ". " + message if message else ""
            return f"Running {event} from {source.id} to {target.id}{message}"

        def on_enter_forward(self):
            print("[SM] ---> FORWARD")
            self.num_cycles = 0
            servo.mid()
            motor.forward()

        def on_exit_forward(self):
            print("[SM] <--- FORWARD")

        def on_enter_left(self):
            print("[SM] ---> TURN LEFT")
            servo.left()
            self.last_distance = 8191
            self.num_cycles = 0

        def on_enter_right(self):
            print("[SM] ---> TURN RIGHT")
            servo.right()
            self.last_distance = 8191
            self.num_cycles = 0

        def on_enter_stopped(self):
            print("[SM] ---> STOPPED")
            servo.mid()
            motor.stop()

        def do_tick(self):
            dist_f = tof_f.get_distance()
            dist_l = tof_l.get_distance()
            dist_r = tof_r.get_distance()
            rawd_f = tof_f.get_raw_distance()
            rawd_l = tof_l.get_raw_distance()
            rawd_r = tof_r.get_raw_distance()
            if DO_CSV:
                print(f"{dist_f},{dist_l},{dist_r},{rawd_f},{rawd_l},{rawd_r}")
            else:
                print(f"F: {dist_f}\tL: {dist_l}\tR: {dist_r}\t|\tf: {rawd_f}\tl: {rawd_l}\tr: {rawd_r}")
            match self.current_state:
                case self.forward:
                    if dist_f < MIN_F_DIST:
                        if dist_l > dist_r:
                            # self.num_cycles = min(1, self.num_cycles + 1)
                            self.num_cycles += 1
                            if self.num_cycles > NUM_CYCLES_F:
                                self.turn_left()
                        elif dist_r > dist_l:
                            # self.num_cycles = min(1, self.num_cycles + 1)
                            self.num_cycles += 1
                            if self.num_cycles > NUM_CYCLES_F:
                                self.turn_right()
                    elif dist_l < MIN_LR_DIST:
                        self.num_cycles += 1
                        if self.num_cycles > NUM_CYCLES_F:
                            self.turn_right()
                    elif dist_r < MIN_LR_DIST:
                        self.num_cycles += 1
                        if self.num_cycles > NUM_CYCLES_F:
                            self.turn_left()
                    else:
                        self.num_cycles = 0
                case self.left:
                    if dist_f < MIN_F_DIST:
                        self.num_cycles = 0
                    elif dist_r > self.last_distance:
                        self.num_cycles += 1
                        if self.num_cycles > NUM_CYCLES_LR:
                            self.go_forward()
                    self.last_distance = dist_r
                case self.right:
                    if dist_f < MIN_F_DIST:
                        self.num_cycles = 0
                    elif dist_l > self.last_distance:
                        self.num_cycles += 1
                        if self.num_cycles > NUM_CYCLES_LR:
                            self.go_forward()
                    self.last_distance = dist_l
                case self.stopped:
                    pass

        def tick_until_done(self):
            while not self.stopped.is_active:
                self.do_tick()
                time.sleep(0.05)

    sm = CarStateMachine()
    sm.tick_until_done()

except KeyboardInterrupt:
    motor.stop()
    for tof in tof_list:
        tof.stop_ranging()
        tof.close()
    GPIO.cleanup()

# cSpell: enable