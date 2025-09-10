#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>

#define EN 25
#define FW 26
#define BW 27
#define ServoPin 18

Servo driveServo;
int servoAngle = 90;
int speed = 255;

void setup() {
    Serial.begin(115200);
    Serial.println("Hello World!");
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH); // Turn the LED on

    pinMode (25, OUTPUT); // EN Pin
    pinMode (FW, OUTPUT); // FW Pin
    pinMode (BW, OUTPUT); // BW Pin

    digitalWrite (EN, LOW); 
    digitalWrite (FW, LOW); 
    digitalWrite (BW, LOW); 

    driveServo.attach(ServoPin);
    driveServo.write(90); // Set the servo to the specified angle
}


void loop() {
// read 1st character from serial input
    char input; 
    if (Serial.available() > 0) {
        int raw = Serial.read();
        if (raw >= 0) {
            input = (char)raw;
        }
        Serial.print("You entered: ");
        Serial.println(input);
        Serial.flush();


        switch (input)
        {
        case 'w':
            digitalWrite(EN,LOW);
            delay(100);
            analogWrite(EN, speed);
            digitalWrite(FW, HIGH);
            digitalWrite(BW, LOW);
            break;

        case 's':
            digitalWrite(EN,LOW);
            delay(100);
            analogWrite(EN, speed);
            digitalWrite(FW, LOW);
            digitalWrite(BW, HIGH);
            break;

        case 'a':
            if (servoAngle < 0) {
                servoAngle += 10;
            }
            driveServo.write(servoAngle);
            Serial.println(servoAngle);
            break;

        case 'd':
            if (servoAngle > 180) {
                servoAngle -= 10;
            }
            driveServo.write(servoAngle);
            Serial.println(servoAngle);
            break;
        
        case 'e':
            if (speed >= 225) {
                speed = 255;
            } else {
                speed += 25;
            }
            Serial.println(speed);
            break;
        case 'q':
            if (speed <= 30) {
                speed = 0;
            } else {
                speed -= 25;
            }
            Serial.println(speed);
            break;
        default:
            digitalWrite(EN, LOW);
            digitalWrite(FW, LOW);
            digitalWrite(BW, LOW);
            break;
        }
    }

    delay(5); // Wait for the servo to reach the position
    // if (input == 'a') {
    //     Serial.println("You pressed 'a'");
    // } else if (input == 'b') {
    //     Serial.println("You pressed 'b'");
    // } else if (input == 'c') {
    //     Serial.println("You pressed 'c'");
    // } else {
    //     Serial.println("You pressed an unrecognized key");
    // }

}
