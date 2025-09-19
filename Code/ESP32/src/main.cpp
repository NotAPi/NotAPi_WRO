#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <TFMPI2C.h>

#define EN 25
#define FW 27
#define BW 26
#define ServoPin 18
#define StartButtonPin 0

#define TF_F 0x14
#define TF_L 0x13
#define TF_R 0x11

TFMPI2C tfm;

uint8_t SENSOR_ADDRS[] = {TF_F, TF_L, TF_R};
const size_t NUM_SENSORS = sizeof(SENSOR_ADDRS) / sizeof(SENSOR_ADDRS[0]);

const int SDA_PIN = 21;
const int SCL_PIN = 22;

const unsigned long TF_READ_PERIOD_MS = 20; // 50 Hz do not go above 100Hz
unsigned long lastTFReadMs = 0;

const unsigned long TF_PRINT_PERIOD_MS = 100; // print every 100 ms
unsigned long lastTFPrintMs = 0;

const char *SENSOR_NAMES[] = {"Front", "Left", "Right"};

Servo driveServo;
int servoAngle = 90;
int speed = 230; // 0-255
bool canStart = false;
int stuckCounter = 0;
const int unstuckSpeed = 255;

const int F_dis_TH = 90;         // cm
const int F_dis_Crash_TH = 35;    // cm
const int Min_Distance_turn = 45; // cm
const int WALL_DIS_TH = 50;       // cm
const int TURN_TIME_MS = 2500;    // ms; minimum time to turn
const int forwardSince = 0;
const int Max_Speed = 200; // cm/s // ik it low but
bool STATUS_LED_STATUS = false;

const int MOTOR_PWM_CH = 8;
const int MOTOR_PWM_FREQ = 5000;
const int MOTOR_PWM_RES = 8; // 8-bit resolution4.34+5.5 mm

int TF_F_DISTANCE;
int TF_L_DISTANCE;
int TF_R_DISTANCE;

const size_t FRONT_DISTANCE_HISTORY = 10;
int frontDistanceHistory[FRONT_DISTANCE_HISTORY] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
size_t frontDistanceHistoryIndex = 0;
size_t frontDistanceHistoryCount = 0;
unsigned long frontDistanceTimeHistory[FRONT_DISTANCE_HISTORY] = {0};

int getDistance(uint8_t addr)
{
    int16_t dist = 0, flux = 0, temp = 0;

    tfm.getData(dist, flux, temp, addr);
    return dist;
}

float getSpeed()
{
    if (frontDistanceHistoryCount < 2)
    {
        return 0.0f;
    }

    size_t newestIndex = (frontDistanceHistoryIndex + FRONT_DISTANCE_HISTORY - 1) % FRONT_DISTANCE_HISTORY;
    size_t oldestIndex = (frontDistanceHistoryCount == FRONT_DISTANCE_HISTORY) ? frontDistanceHistoryIndex : 0;

    unsigned long newestTime = frontDistanceTimeHistory[newestIndex];
    unsigned long oldestTime = frontDistanceTimeHistory[oldestIndex];

    if (newestTime <= oldestTime)
    {
        return 0.0f;
    }

    int newestDistance = frontDistanceHistory[newestIndex];
    int oldestDistance = frontDistanceHistory[oldestIndex];

    float distanceDelta = static_cast<float>(oldestDistance - newestDistance);
    float timeDeltaSeconds = (newestTime - oldestTime) / 1000.0f;

    return distanceDelta / timeDeltaSeconds;
}

void setSpeed(int spd)
{
    // Serial.print("Setting speed to ");
    // Serial.println(spd);
    if (spd < 0)
        spd = 0;
    if (spd > 255)
        spd = 255;

    if (getDistance(TF_F) > Min_Distance_turn)
    {
        bool forwardActive = digitalRead(FW) == HIGH && digitalRead(BW) == LOW;
        if (forwardActive && spd > 0)
        {
            float currentSpeed = getSpeed();
            if (currentSpeed > Max_Speed)
            {
                spd = 0;
                Serial.println("Speed limit exceeded");
            }
        }
    }
    // Serial.print("Set speed to ");
    // Serial.println(spd);
    ledcWrite(EN, spd);
}

void forward()
{
    Serial.println("FORWARD");
    setSpeed(speed);
    digitalWrite(FW, HIGH);
    digitalWrite(BW, LOW);
}

void backward()
{
    digitalWrite(FW, LOW);
    digitalWrite(BW, HIGH);
}

void stop()
{
    digitalWrite(FW, LOW);
    digitalWrite(BW, LOW);
}

void turn(int angle) // for int
{
    Serial.println(angle);
    if (angle < 60)
        angle = 60;
    if (angle > 120)
        angle = 120;
    driveServo.write(angle);
}

void turn(char dir) // for char
{
    if (dir == 'L')
        turn(120);
    else if (dir == 'R')
        turn(60);
    else
        turn(90);
}

bool checkIfStuck()
{
    if (frontDistanceHistoryCount < 2)
    {
        return false;
    }

    size_t newestIndex = (frontDistanceHistoryIndex + FRONT_DISTANCE_HISTORY - 1) % FRONT_DISTANCE_HISTORY;
    size_t oldestIndex = (frontDistanceHistoryCount == FRONT_DISTANCE_HISTORY) ? frontDistanceHistoryIndex : 0;

    int newestDistance = frontDistanceHistory[newestIndex];
    int oldestDistance = frontDistanceHistory[oldestIndex];

    if ((newestDistance + oldestDistance) / 2 < 12)
    {
        return true;
    }
    return false;
}

void setup()
{
    ESP32PWM::allocateTimer(1); // PWM timer 1
    ESP32PWM::allocateTimer(2); // PWM timer 2
    ESP32PWM::allocateTimer(3); // PWM timer 3
    Serial.begin(115200);
    Serial.println("Hello World!");
    delay(1000);
    // driveServo.attach(ServoPin);
    if (driveServo.attach(ServoPin) < 0) // some servo fix ?
    {
        driveServo.detach();
        ledcDetach(ServoPin);
        delay(100);
        driveServo.attach(ServoPin);
    }
    delay(2000);
    driveServo.write(90); // Center the servo
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH); // Turn the LED on

    // Initialize I2C bus and recover if needed
    tfm.recoverI2CBus(SDA_PIN, SCL_PIN);
    Wire.setClock(400000); // 400kHz I2C
    delay(100);
    digitalWrite(LED_BUILTIN, LOW); // Turn the LED off

    // Send example commands to all TFMini-Plus sensors
    for (size_t i = 0; i < NUM_SENSORS; ++i)
    {
        uint8_t addr = SENSOR_ADDRS[i];

        Serial.print("Sensor @0x");
        if (addr < 16)
            Serial.print('0');
        Serial.print(addr, HEX);
        Serial.print(" - System reset: ");

        if (tfm.sendCommand(SOFT_RESET, 0, addr))
        {
            Serial.println("passed.");
        }
        else
        {
            tfm.printReply();
        }

        delay(50);

        Serial.print("Sensor @0x");
        if (addr < 16)
            Serial.print('0');
        Serial.print(addr, HEX);
        Serial.print(" - Firmware version: ");

        if (tfm.sendCommand(GET_FIRMWARE_VERSION, 0, addr))
        {
            Serial.print(tfm.version[0]);
            Serial.print('.');
            Serial.print(tfm.version[1]);
            Serial.print('.');
            Serial.println(tfm.version[2]);
        }
        else
        {
            tfm.printReply();
        }
    }
    delay(10);
    digitalWrite(LED_BUILTIN, HIGH); // Turn the LED on

    pinMode(EN, OUTPUT);                                      // EN Pin
    ledcAttachChannel(EN, MOTOR_PWM_FREQ, MOTOR_PWM_RES, 10); // EN Pin
    pinMode(FW, OUTPUT);                                      // FW Pin
    pinMode(BW, OUTPUT);                                      // BW Pin
    pinMode(StartButtonPin, INPUT_PULLDOWN);

    digitalWrite(EN, LOW);
    digitalWrite(FW, LOW);
    digitalWrite(BW, LOW);

    digitalWrite(LED_BUILTIN, LOW); // Turn the LED off
}

void loop()
{
    // Non-blocking periodic read/print of all TF-Mini sensors every second
    if (digitalRead(StartButtonPin) == LOW && !canStart)
    {
        canStart = true;
        setSpeed(speed);
        digitalWrite(LED_BUILTIN, HIGH);
        delay(2000);
        turn(90);
    }
    else if (digitalRead(StartButtonPin) == LOW && canStart)
    {
        canStart = false;
        stop();
        setSpeed(0);
        digitalWrite(LED_BUILTIN, LOW);
        delay(2000);
        turn(90);
    }

    // CAR LOGIC ALGORITHM:
    // START:
    // F_dis > F_dis_TH -> forward
    // F_dis < F_dis_TH -> stop -> L_dis > R_dis ? turn left : turn right
    // DO PID to try to keep OUTER WALL distance = WALL_DIS_TH
    // LOOP
    if (!canStart)
    {
        STATUS_LED_STATUS = !STATUS_LED_STATUS;
        delay(100);
        digitalWrite(LED_BUILTIN, STATUS_LED_STATUS); // Turn the LED off
        // return;
    }
    else
    {
        unsigned long now = millis();

        if (now - lastTFReadMs >= TF_READ_PERIOD_MS)
        {
            lastTFReadMs = now;
            size_t historyCountBeforeUpdate = frontDistanceHistoryCount;
            int prevFrontMaxDistance = -1;
            if (frontDistanceHistoryCount > 0)
            {
                prevFrontMaxDistance = frontDistanceHistory[0];
                for (size_t i = 1; i < frontDistanceHistoryCount; ++i)
                {
                    if (frontDistanceHistory[i] > prevFrontMaxDistance)
                    {
                        prevFrontMaxDistance = frontDistanceHistory[i];
                    }
                }
            }
            TF_F_DISTANCE = getDistance(TF_F);
            TF_L_DISTANCE = getDistance(TF_L);
            TF_R_DISTANCE = getDistance(TF_R);

            frontDistanceHistory[frontDistanceHistoryIndex] = TF_F_DISTANCE;
            frontDistanceTimeHistory[frontDistanceHistoryIndex] = now;
            frontDistanceHistoryIndex = (frontDistanceHistoryIndex + 1) % FRONT_DISTANCE_HISTORY;
            if (frontDistanceHistoryCount < FRONT_DISTANCE_HISTORY)
            {
                frontDistanceHistoryCount++;
            }

            Serial.print("F: ");
            Serial.print(TF_F_DISTANCE);
            Serial.print(" L: ");
            Serial.print(TF_L_DISTANCE);
            Serial.print(" R: ");
            Serial.print(TF_R_DISTANCE);
            Serial.print(" Speed: ");
            Serial.print(getSpeed());
            Serial.println(" cm/s");

            if (checkIfStuck())
            {
                Serial.println("POSSIBLY STUCK");

                while (true)
                {
                    Serial.print("Speed history: ");
                    for (size_t i = 0; i < frontDistanceHistoryCount; ++i)
                    {
                        Serial.print(frontDistanceHistory[i]);
                        Serial.print(" ");
                    }
                    Serial.println();
                    delay(1000);
                }
            }

            if (TF_F_DISTANCE < F_dis_Crash_TH)
            {
                int stuck = 0;
                Serial.println("CRASH! STOP");
                backward();
                setSpeed(speed);
                delay(100);
                int startTime = millis();
                while (getDistance(TF_F) < Min_Distance_turn)
                {
                    int prevDistance = getDistance(TF_F);
                    delay(50);
                    // if (getSpeed() < 10) // if not getting away, break
                    // {
                    //     setSpeed(255);
                    // }

                    if (millis() - startTime > 8000)
                    {
                        canStart = false; // if stuck for 8s, stop
                    }
                    else if (millis() - startTime > 2000)
                    {
                        // setSpeed(255); // after 2s, go full speed
                        backward();
                        delay(500);
                        forward();
                        delay(500);
                        backward();
                        delay(500);

                        stuck++;
                    }
                    if (stuck != 0)
                    {
                        backward();
                        delay(500);
                    }
                }

                if (stuck == 1)
                {
                }
                stop();
                setSpeed(0);
                turn(90);
                // canStart = false;
                digitalWrite(LED_BUILTIN, LOW);
                delay(500);
                return; // wait for manual restart
            }
            // else if (historyCountBeforeUpdate == FRONT_DISTANCE_HISTORY &&
            //          prevFrontMaxDistance - TF_F_DISTANCE > 1 /*&& forwardSince > 3000*/) // possibly stuck detect
            // {
            //     Serial.println("POSSIBLY STUCK");
            //     stuckCounter++;
            //     if (stuckCounter > 3)
            //     {
            //         Serial.println("STUCK! ");
            //         stuckCounter = 3; // max 3
            //         while (prevFrontMaxDistance - TF_F_DISTANCE > 2)
            //         {
            //             Serial.println("STUCK! UNSTUCKING");
            //             TF_F_DISTANCE = getDistance(TF_F);
            //             delay(50);
            //             setSpeed(unstuckSpeed);
            //             backward();
            //             delay(500);
            //             forward();
            //             delay(500);
            //             backward();
            //             delay(1000);
            //             setSpeed(speed);
            //         }
            //         stuckCounter = 0;
            //     }
            // return;
            // }

            if (TF_F_DISTANCE > F_dis_TH)
            {

                forward();
                setSpeed(speed);
                // if (TF_L_DISTANCE < WALL_DIS_TH)
                // {
                //     turn(120); // turn right
                // }
                // else if (TF_R_DISTANCE < WALL_DIS_TH)
                // {
                //     turn(60); // turn left
                // }
                // else
                // {
                turn(90); // go straight
                // }
            }
            else // too close to front wall, turn
            {
                char turnDirection;
                // stop();
                backward();
                setSpeed(200);
                delay(1000);
                while (getDistance(TF_F) < Min_Distance_turn)
                {
                    int prevDistance = getDistance(TF_F);
                    delay(50);
                    if (prevDistance - getDistance(TF_F) < 5) // if not getting away, break
                        break;
                }

                // delay(100);
                int prevTurnDistance = getDistance(TF_F);
                if (TF_L_DISTANCE > TF_R_DISTANCE) //&& TF_L_DISTANCE > WALL_DIS_TH)
                {
                    turn('L'); // turn left
                    Serial.println("TURN LEFT");
                    turnDirection = 'L';
                }
                else
                {
                    turn('R'); // turn right
                    Serial.println("TURN RIGHT");
                    turnDirection = 'R';
                }
                // delay(100);
                forward();
                setSpeed(speed);
                delay(TURN_TIME_MS);

                if (getDistance(TF_F) == prevTurnDistance) // STUCK
                {
                    Serial.println("STUCK IN TURN, TRY AGAIN");
                    backward();
                    while (getDistance(TF_F) - prevTurnDistance < 5)
                    {
                        backward();
                        delay(500);
                        forward();
                        delay(500);
                    }
                }

                // // turn until L and R are similar distance (10% tolerance) or timeout 2s
                // unsigned long startTime = millis();
                // while (millis() - startTime < 2000)
                // {
                //     TF_L_DISTANCE = getDistance(TF_L);
                //     TF_R_DISTANCE = getDistance(TF_R);
                //     if (abs(TF_L_DISTANCE - TF_R_DISTANCE) < 0.2 * TF_R_DISTANCE)
                //     {
                //         break;
                //     }
                // }

                // continue turning until the outer sensor distance starts to increase
                int outerSensor = (turnDirection == 'L') ? TF_R : TF_L;
                int previousOuterDistance = getDistance(outerSensor);
                while (true)
                {
                    int currentOuterDistance = getDistance(outerSensor);
                    if (currentOuterDistance > previousOuterDistance)
                    {
                        break;
                    }
                    previousOuterDistance = currentOuterDistance;
                }

                // try to correct the angle a bit
                setSpeed(180);
                // if (turnDirection == 'L')
                //     turn('R'); // turn left
                // else
                //     turn('L'); // turn right
                delay(20);
                turn(90); // go straight
            }

            // for ()
            // {
            //     uint8_t addr = SENSOR_ADDRS[i];
            //     int distance = getDistance(addr);

            //     Serial.print(SENSOR_NAMES[i]);
            //     Serial.print(" (0x");
            //     if (addr < 16)
            //         Serial.print('0');
            //     Serial.print(addr, HEX);
            //     Serial.print("): ");
            //     if (distance >= 0)
            //     {
            //         Serial.print(distance);
            //         Serial.println(" mm");
            //     }
            //     else
            //     {
            //         Serial.println("Error");
            //     }
            // }
            // Serial.println();
        }
    }
    // THIS IS FOR DEBUGGING VIA SERIAL MONITOR, (RC CAR MODE)
    char input;
    if (Serial.available() > 0)
    {
        int raw = Serial.read();
        if (raw >= 0)
        {
            input = (char)raw;
        }
        Serial.print("You entered: ");
        Serial.println(input);
        Serial.flush();

        switch (input)
        {
        case 'w':
            setSpeed(0);
            delay(100);
            forward();
            setSpeed(speed);
            break;

        case 's':
            setSpeed(0);
            delay(100);
            backward();
            setSpeed(speed);
            break;

        case 'a':
            if (servoAngle < 120)
            {
                servoAngle += 10;
            }
            driveServo.write(servoAngle);
            Serial.println(servoAngle);
            break;

        case 'd':
            if (servoAngle > 60)
            {
                servoAngle -= 10;
            }
            driveServo.write(servoAngle);
            Serial.println(servoAngle);
            break;

        case 'e':
            if (speed >= 10025)
            {
                speed = 255;
            }
            else
            {
                speed += 25;
            }
            setSpeed(speed);
            Serial.println(speed);
            break;
        case 'q':
            if (speed <= 30)
            {
                speed = 0;
            }
            else
            {
                speed -= 25;
            }
            setSpeed(speed);
            Serial.println(speed);
            break;
        case 'p':
            if (!canStart)
            {
                canStart = true;
                setSpeed(speed);
                digitalWrite(LED_BUILTIN, HIGH);
                delay(2000);
                turn(90);
            }
            else if (canStart)
            {
                canStart = false;
                stop();
                setSpeed(0);
                digitalWrite(LED_BUILTIN, LOW);
                delay(2000);
                turn(90);
            }
        default:
            setSpeed(0);
            stop();
            break;
        }
    }

    delay(5);
}
