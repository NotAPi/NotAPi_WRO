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

const unsigned long READ_PERIOD_MS = 20; // 50 Hz do not go above 100Hz
unsigned long lastReadMs = 0;

const unsigned long TF_PRINT_PERIOD_MS = 100; // print every 100 ms
unsigned long lastTFPrintMs = 0;

const char *SENSOR_NAMES[] = {"Front", "Left", "Right"};

Servo driveServo;
int servoAngle = 90;
int speed = 255;
bool canStart = false;

const int F_dis_TH = 130; // cm
const int WALL_DIS_TH = 30; // cm 
const int TURN_TIME_MS = 2000; // ms

bool STATUS_LED_STATUS = false;

const int MOTOR_PWM_CH = 8;
const int MOTOR_PWM_FREQ = 5000;
const int MOTOR_PWM_RES = 8; // 8-bit resolution4.34+5.5 mm

int TF_F_DISTANCE;
int TF_L_DISTANCE;
int TF_R_DISTANCE;

int getDistance(uint8_t addr)
{
    int16_t dist = 0, flux = 0, temp = 0;

    tfm.getData(dist, flux, temp, addr);
    return dist;
}

void forward()
{
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

void setSpeed(int spd)
{
    ledcWrite(EN, spd);
}

void turn(int angle)
{
    if (angle < 60)
        angle = 60;
    if (angle > 120)
        angle = 120;
    driveServo.write(angle);
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
        driveServo.detach(); ledcDetach(ServoPin); delay(100);
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


    pinMode(EN, OUTPUT); // EN Pin
    ledcAttachChannel(EN, MOTOR_PWM_FREQ, MOTOR_PWM_RES, 10); // EN Pin
    pinMode(FW, OUTPUT); // FW Pin
    pinMode(BW, OUTPUT); // BW Pin
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
    } else if (digitalRead(StartButtonPin) == LOW && canStart)
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
    } else {
    unsigned long now = millis();

    if (now - lastTFPrintMs >= TF_PRINT_PERIOD_MS)
    {
        lastTFPrintMs = now;

        TF_F_DISTANCE = getDistance(TF_F);
        TF_L_DISTANCE = getDistance(TF_L);
        TF_R_DISTANCE = getDistance(TF_R);

        Serial.print("F: ");
        Serial.print(TF_F_DISTANCE);
        Serial.print(" L: ");
        Serial.print(TF_L_DISTANCE);
        Serial.print(" R: ");
        Serial.print(TF_R_DISTANCE);
        Serial.println();

        if (TF_F_DISTANCE > F_dis_TH)
        {
            forward();
            setSpeed(speed);
            if (TF_L_DISTANCE < WALL_DIS_TH)
            {
                turn(120); // turn right
            }
            else if (TF_R_DISTANCE < WALL_DIS_TH)
            {
                turn(60); // turn left
            }
            else
            {
                turn(90); // go straight
            }
        }
        else // too close to front wall, turn
        {
            // stop();
            delay(100);
            if (TF_L_DISTANCE > TF_R_DISTANCE)
            {
                turn(60); // turn left
                Serial.println("TURN LEFT");
            }
            else
            {
                turn(120); // turn right
                Serial.println("TURN RIGHT");
            }
            // delay(100);
            forward();
            setSpeed(speed);
            delay(TURN_TIME_MS);
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
            ledcWrite(EN, 0);
            delay(100);
            ledcWrite(EN, speed);
            digitalWrite(FW, HIGH);
            digitalWrite(BW, LOW);
            break;

        case 's':
            ledcWrite(EN, 0);
            delay(100);
            ledcWrite(EN, speed);
            digitalWrite(FW, LOW);
            digitalWrite(BW, HIGH);
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
            ledcWrite(EN, speed);
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
            ledcWrite(EN, speed);
            Serial.println(speed);
            break;
        default:
            ledcWrite(EN, 0);
            digitalWrite(FW, LOW);
            digitalWrite(BW, LOW);
            break;
        }
    }

    delay(5); 
}