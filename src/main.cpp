#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include "Movement.cpp"
#include "Constants.cpp"

// put function declarations here:
// void defaultPosition(int, int);

// Declare robot as a global variable
Robot* robot = nullptr;
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);

void setup() {
  // put your setup code here, to run once:
  // int result = myFunction(2, 3);
  Serial.begin(9600);
  Serial.println("Hexapod Code");
  robot = new Robot(0, 0, 0); // Initialize the robot with coordinates (0, 0, 0)
  pwm1.begin();
  pwm1.setPWMFreq(60);
  pwm2.begin();
  pwm2.setPWMFreq(60);
  delay(1000);
  robot -> defaultPosition();   // Robot initialization to default configurations
  Serial.println("Robot initialized to default position.");
}

void loop() { // Reads the user's input from the keyboard's arrow keys and controls the robot accordingly.
  int input = Serial.read();
  // Leg& legA = robot -> leg0; // left front leg, make sure to call by reference  
  // Leg& legB = robot -> leg1; // left front leg, make sure to call by reference
  // Leg& legC = robot -> leg2; // left middle leg
  // Leg& legD = robot -> leg3; // left front leg, make sure to call by reference
  Leg& legE = robot -> leg4; // left middle leg
  // Leg& legF = robot -> leg5; // left middle leg
  if (robot == nullptr) {
    Serial.println("Robot not initialized");
    return; // Exit if robot is not initialized
  }

  if (input == 99) { // "c" (for testing)
    // Serial.println(robot.currXCoord);
    robot -> robotMove(0, SPEED, 0); // right walk
    // legA.moveFemur(10);
    // legB.moveFemur(10);
    // legC.moveFemur(10);
    // legD.moveFemur(10);
    // legE.moveFemur(10);
    // legF.moveFemur(10);

    // int size = sizeof(legA.interpolationZResults) / sizeof(legA.interpolationZResults[0]);
    // for (int i = 0; i < size; i++) {
    //   legA.calculateIK(robot.currXCoord, robot.currYCoord, robot.currZCoord, legA.interpolationXResults[i],
    //     legA.interpolationYResults[i], legA.interpolationZResults[i]); // TODO: change this to use the x and y coordinates
    // }
  }

  if (input == 101) { // "e" (for testing)
    // Serial.println(robot.currXCoord);
    // robot -> robotMove(10, 0, 0); // right walk
    // legA.moveCoxa(10);
    // legB.moveCoxa(10);
    // legC.moveCoxa(10);
    // legD.moveCoxa(10);
    legE.moveTibia(-10);
  }
  
  if (input == 119) { // "w"
    robot -> robotMove(0, SPEED, 0);
  } else if (input == 97) { // "a"
    robot -> robotMove(-SPEED, 0, 0);
  } else if (input == 115) { // "s"
    robot -> robotMove(0, -SPEED, 0);
  } else if (input == 100) { // "d"
    robot -> robotMove(SPEED, 0, 0);
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
