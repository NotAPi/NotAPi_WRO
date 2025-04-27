#!/usr/bin/python

import time
import VL53L0X
import RPi.GPIO as GPIO

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

tof_list = [
    VL53L0X_XSHUT(i2c_address=0x2a, xshut_pin=22),
    VL53L0X_XSHUT(i2c_address=0x2b, xshut_pin=24),
    VL53L0X_XSHUT(i2c_address=0x2c, xshut_pin=25),
]

for tof in tof_list:
    tof.open()
    tof.start_ranging(VL53L0X.Vl53l0xAccuracyMode.BETTER)

timing = tof_list[0].get_timing()
if timing < 20000:
    timing = 20000
print("Timing %d ms" % (timing/1000))

for count in range(1,101):
    for idx, tof in enumerate(tof_list):
        distance = tof.get_distance()
        if distance > 0:
            print("sensor %d - %d mm, %d cm, iteration %d" % (idx, distance, (distance / 10), count))
        else:
            print("%d - Error" % idx)

    print()
    time.sleep(timing/1000000.00)

for tof in tof_list:
    tof.stop_ranging()
    tof.close()
