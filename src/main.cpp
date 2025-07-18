#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include "Movement.cpp"
#include "Constants.cpp"
#include "Vector3.h"

// Declare robot as a global variable
Robot* robot = nullptr;
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);

void setup() {
  Serial.begin(9600);
  Serial.println("Hexapod Code");
  
  // Initialize the robot with coordinates (0, 0, 0)
  robot = new Robot(0, 0, 0);
  pwm1.begin();
  pwm1.setPWMFreq(60);
  pwm2.begin();
  pwm2.setPWMFreq(60);
  delay(1000);
  
  // Robot initialization to default configurations
  robot -> defaultPosition();
  Serial.println("Robot initialized to default position.");
}

// Reads the user's input from the keyboard's arrow keys and controls the robot accordingly.
void loop() {
  int input = Serial.read();

  // Exit if robot is not initialized
  if (robot == nullptr) {
    Serial.println("Robot not initialized");
    return;
  }

  // "c" (for testing)
  if (input == 99) { 
    robot -> robotMove(Vector3(0, SPEED, 0)); // right walk
  }

  // "z" (for testing)
  if (input == 122) {
    robot -> defaultPosition();
  }
  
  if (input == 119) { // "w"
    robot -> robotMove(Vector3(0, SPEED, 0));
  } else if (input == 97) { // "a"
    robot -> robotMove(Vector3(-SPEED, 0, 0));
  } else if (input == 115) { // "s"
    robot -> robotMove(Vector3(0, -SPEED, 0));
  } else if (input == 100) { // "d"
    robot -> robotMove(Vector3(SPEED, 0, 0));
  }

  if (input == 49) { // 1: mode 1: can do the lethal dance
    robot -> mode1();
  } else if (input == 50) { // 2: mode 2: turn around in place
    robot -> mode2();
  } else if (input == 51) { // 3: mode 3: jump?
    robot -> mode3();
  } else if (input == 52) { // 4: mode 4: do a wave
    robot -> mode4();
  }
}
