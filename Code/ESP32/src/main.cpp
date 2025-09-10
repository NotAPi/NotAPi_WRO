#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>

#define EN 25
#define FW 26
#define BW 27

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

        if (input == 'w') {
            digitalWrite(EN,LOW);
            delay(100);
            
            digitalWrite(EN, HIGH);
            digitalWrite(FW, HIGH);
            digitalWrite(BW, LOW);

        } else if (input == 's') {
            digitalWrite(EN,LOW);
            delay(100);

            digitalWrite(EN, HIGH);
            digitalWrite(FW, LOW);
            digitalWrite(BW, HIGH);
        } else
        {
            digitalWrite(EN, LOW);
            digitalWrite(FW, LOW);
            digitalWrite(BW, LOW);
        }

    }

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
