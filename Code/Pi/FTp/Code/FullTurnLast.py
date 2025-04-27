import time
import pigpio
import os

try:
    os.system("sudo pigpiod")  # Launching GPIO library
    time.sleep(1)  # As it takes some time to launch
except:
    #aaaprint("GPIO library already launched")
    pass

# 55------105---155

# Initialize pigpio
pi = pigpio.pi()

L_RX_PIN = 22 #left recieve pin son serial
R_RX_PIN = 24
F_RX_PIN = 25

IN1_PIN = 10 #motor pins
IN2_PIN = 11
ENA_PIN = 12

SERVO_PIN = 17 #servo pin

BUTTON_PIN = 16 #button pin

Laps = 0
# Try to close the serial connection if it is already open
try:
    pi.bb_serial_read_close(R_RX_PIN) 
    pi.bb_serial_read_close(L_RX_PIN)
    pi.bb_serial_read_close(F_RX_PIN)
except:
    pass

# Set up the button pin as input with a pull-up resistor
    pi.set_mode(BUTTON_PIN, pigpio.INPUT)
    pi.set_pull_up_down(BUTTON_PIN, pigpio.PUD_UP)

# Set up the software serial connection
pi.set_mode(R_RX_PIN, pigpio.INPUT) 
pi.bb_serial_read_open(R_RX_PIN, 115200) 
pi.set_mode(L_RX_PIN, pigpio.INPUT) 
pi.bb_serial_read_open(L_RX_PIN, 115200)
pi.set_mode(F_RX_PIN, pigpio.INPUT) 
pi.bb_serial_read_open(F_RX_PIN, 115200)
#aaaprint("init done")

def parse_lidar_data(data): 
    if len(data) >= 9 and data[0] == 0x59 and data[1] == 0x59:
        distance = data[2] + data[3] * 256
        strength = data[4] + data[5] * 256
        return distance, strength
    return None, None

def F_read_lidar():
    while True:
        (count, data) = pi.bb_serial_read(F_RX_PIN)
        if count > 0:
            break
    if count > 0:
        distance, strength = parse_lidar_data(data)
        if distance is not None: # Isn't really needed
            # #aaaprint(f"DistanceR: {distance} cm, Strength: {strength}")
            return distance

def R_read_lidar():
    while True:
        (count, data) = pi.bb_serial_read(R_RX_PIN)
        if count > 0:
            break
    if count > 0:
        distance, strength = parse_lidar_data(data)
        if distance is not None: # Isn't really needed
            # #aaaprint(f"DistanceR: {distance} cm, Strength: {strength}")
            return distance

def L_read_lidar():
    while True:
        (count, data) = pi.bb_serial_read(L_RX_PIN)
        if count > 0:
            break
    if count > 0:
        distance, strength = parse_lidar_data(data)
        if distance is not None: # Isn't really needed
            # #aaaprint(f"DistanceL: {distance} cm, Strength: {strength}")
            return distance
        
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

def servo(angle=105):
    pulse_width = 500 + (angle / 180.0) * 2000
    pi.set_servo_pulsewidth(SERVO_PIN, pulse_width)

def turnRight(angle=90):
    stop()
    time.sleep(0.05)
    if angle == 90:
        servo(155)
        forward(255)
        time.sleep(1.95)

def turnLeft(angle=90):
    stop()
    time.sleep(0.05)
    if angle == 90:
        servo(55)
        forward(255)
        time.sleep(1.95)

def forwardm(distance=1):
    forward(255)
    time.sleep(6.6*distance)

def distances():
    
    # Get Front distance    
    Fdistance = F_read_lidar()
    while Fdistance is None or Fdistance > 500 or Fdistance <= 0:
        Fdistance = F_read_lidar()
        if Fdistance is not None and Fdistance < 500 and Fdistance > 0:
            if Fdistance > 100:
                Fdistance2 = F_read_lidar()
                if abs(Fdistance - Fdistance2) > 20:
                    Fdistance = min(Fdistance2, Fdistance)

    # Get Left distance    
    Ldistance = L_read_lidar()
    while Ldistance is None or Ldistance > 500 or Ldistance <= 0:
        Ldistance = L_read_lidar()
        if Ldistance is not None and Ldistance < 500 and Ldistance > 0:
            if Ldistance > 100:
                Ldistance2 = L_read_lidar()
                if abs(Ldistance - Ldistance2) > 20:
                    Ldistance = min(Ldistance2, Ldistance)
                    break
                
    # Get Right distance    
    Rdistance = R_read_lidar()
    while Rdistance is None or Rdistance > 500 or Rdistance <= 0:
        Rdistance = R_read_lidar()
        if Rdistance is not None and Rdistance < 500 and Rdistance > 0:
            if Rdistance > 100:
                Rdistance2 = R_read_lidar()
                if abs(Rdistance - Rdistance2) > 20:
                    Rdistance = min(Rdistance2, Rdistance)

    # Ensure all distances are valid before returning
    if Ldistance is None or Rdistance is None or Fdistance is None:
        raise ValueError("One or more distance readings are invalid")

    return Ldistance, Rdistance, Fdistance

def turnLeftFull():
    servo()
    forwardm(0.2)
    turnLeft(90)
    
    servo()
    # forwardm(0.5)

    stop()
    time.sleep(0.05)
    #aaaprint("read")
    forward()
    time.sleep(0.1)
    Ldistance, Rdistance, Fdistance = distances()
    #aaaprint("read done")
    stop()
    #aaaprint("L " + str(Ldistance) + " R " + str(Rdistance) + " SUM " + str(Ldistance + Rdistance) + " F " + str(Fdistance))

    while Ldistance + Rdistance > 150:
        forward()
        Ldistance, Rdistance, Fdistance = distances()
        #aaaprint("L " + str(Ldistance) + " R " + str(Rdistance) + " SUM " + str(Ldistance + Rdistance) + " F " + str(Fdistance))
        # forward()        
        time.sleep(0.2)
    #aaaprint("Stop")
    time.sleep(0.05)    
    stop()
    
def turnRightFull():
    servo(105)
    forwardm(0.2)
    turnRight(90)
    
    servo()
    # forwardm(0.5)

    stop()
    time.sleep(0.05)
    #aaaprint("read")
    forward()
    time.sleep(0.1)
    Ldistance, Rdistance, Fdistance = distances()
    #aaaprint("read done")
    stop()
    #aaaprint("L " + str(Ldistance) + " R " + str(Rdistance) + " SUM " + str(Ldistance + Rdistance) + " F " + str(Fdistance))

    while Ldistance + Rdistance > 150:
        forward()
        Ldistance, Rdistance, Fdistance = distances()
        #aaaprint("L " + str(Ldistance) + " R " + str(Rdistance) + " SUM " + str(Ldistance + Rdistance) + " F " + str(Fdistance))
        # forward()        
        time.sleep(0.2)
    #aaaprint("Stop")
    time.sleep(0.05)    
    stop()

def checkTime(lastTurn):
    if time.time() - lastTurn > 20:
        backward(255)
        time.sleep(1)
        if Left:
            servo(75)
            forward()
            time.sleep(0.5)
        if Right:
            servo(135)
            forward()
            time.sleep(0.5)    
            
        servo()
    
try:
    Left = 0
    Right = 0
    lastTurn = 0
    servo()
    time.sleep(0.05)
    
    
    # Wait until the button is pressed (connected to GND)
    while pi.read(BUTTON_PIN) != 0:
        print("Waiting for button")
        print(pi.read(BUTTON_PIN))
        time.sleep(0.1)
        time.sleep(0.1)
    forward()
    giros = 0
    while True:
        Ldistance, Rdistance, Fdistance = distances()
        Ldistance = int(Ldistance)
        Rdistance = int(Rdistance)
        Fdistance = int(Fdistance)
        lastTurn = time.time()
        while Ldistance < 100 or Rdistance < 100:
            checkTime(lastTurn)
            forward()
            Ldistance, Rdistance, Fdistance = distances()
            Ldistance = int(Ldistance)
            Rdistance = int(Rdistance)
            Fdistance = int(Fdistance)
            print("L " + str(Ldistance) + " R " + str(Rdistance) + " SUM " + str(int(Ldistance) + int(Rdistance)) + " F " + str(Fdistance))
            time.sleep(0.2)
            if Ldistance + Rdistance > 110:
                break
        #aaaprint("Turn time")
        
        if Ldistance > Rdistance:
            turnLeftFull()
            Left = 1
            giros = giros + 1
            #aaaprint("Left")
        
        if Rdistance > Ldistance:
            turnRightFull()
            Right = 1
            giros = giros + 1
            #aaaprint("Right")
        forwardm(0.25)
        forward()
        lastTurn = time.time() 
        Ldistance, Rdistance, Fdistance = distances()
        Ldistance = int(Ldistance)
        Rdistance = int(Rdistance)
        Fdistance = int(Fdistance)

        while int(Fdistance) > 165:
            checkTime(lastTurn)
            forward()
            Ldistance, Rdistance, Fdistance = distances()
            Ldistance = int(Ldistance)
            Rdistance = int(Rdistance)
            Fdistance = int(Fdistance)
            print("L " + str(Ldistance) + " R " + str(Rdistance) + " SUM " + str(Ldistance + Rdistance) + " F " + str(Fdistance))
            time.sleep(0.2)
            #aaaprint ("F " + str(Fdistance))
            if int(Fdistance) < 165:
                break
        
        if giros == 12:
            break
        #aaaprint("150")
        
        #aaaprint("Correct time")
        Ldistance, Rdistance, Fdistance = distances()
        Ldistance = int(Ldistance)
        Rdistance = int(Rdistance)
        Fdistance = int(Fdistance)
        #aaaprint("L " + str(Ldistance) + " R " + str(Rdistance) + " SUM " + str(Ldistance + Rdistance) + " F " + str(Fdistance))
        stop()
        if abs(Ldistance - Rdistance) > 8:
            if Ldistance - Rdistance:
                servo(95)  # Turn left
                #aaaprint("90")
            elif Rdistance - Ldistance:
                servo(115)  # Turn right
                #aaaprint("120")
            else:
                servo()  # Go straight
                #aaaprint("105")
        forwardm(0.1)
        servo()
    
        for a in range(5):
            print(giros)
        lastTurn = time.time()
        
    forwardm(0.5)
    stop()

        
        

except KeyboardInterrupt:
    stop()
    pi.bb_serial_read_close(R_RX_PIN)  # Close the serial connection on exit
    pi.bb_serial_read_close(L_RX_PIN)  # Close the serial connection on exit
    pi.bb_serial_read_close(F_RX_PIN)  # Close the serial connection on exit

    pi.stop()