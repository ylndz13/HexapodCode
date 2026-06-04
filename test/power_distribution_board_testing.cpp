#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);

void setup() {
    Serial.begin(9600);
    Serial.println("pdb testing");
    pwm1.begin();
    pwm1.setPWMFreq(60);
    delay(1000);
}

void loop() {
    int servonum = 8;
    int angle = 90;
    pwm1.setPWM(servonum, 0, 500);
}