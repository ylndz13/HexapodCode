#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>


// // // servo numbers correspond to numbers on servo holder, not the PCA pins
// #define SERVO0_MIN 150 
// #define SERVO0_MAX 700
// #define SERVO1_MIN 130
// #define SERVO1_MAX 700
// #define SERVO2_MIN 150
// #define SERVO2_MAX 700
// #define SERVO3_MIN 200
// #define SERVO3_MAX 600
// #define SERVO4_MIN 120
// #define SERVO4_MAX 600
// #define SERVO5_MIN 150
// #define SERVO5_MAX 610
// #define SERVO6_MIN 120
// #define SERVO6_MAX 620
// #define SERVO7_MIN 150
// #define SERVO7_MAX 670
// #define SERVO8_MIN 130
// #define SERVO8_MAX 650
// #define SERVO9_MIN 150
// #define SERVO9_MAX 620
// #define SERVO10_MIN 180
// #define SERVO10_MAX 640
// #define SERVO11_MIN 130
// #define SERVO11_MAX 600
// #define SERVO12_MIN 110
// #define SERVO12_MAX 630
// #define SERVO13_MIN 150
// #define SERVO13_MAX 600 //computer plug is blocking it... actual value around 640?
// #define SERVO14_MIN 150
// #define SERVO14_MAX 660
// #define SERVO15_MIN 140
// #define SERVO15_MAX 600
// #define SERVO16_MIN 110 
// #define SERVO16_MAX 510
// #define SERVO17_MIN 120
// #define SERVO17_MAX 580


// // //PWM1
// // uint8_t servonum5 = 9;
// // uint8_t servonum11 = 10;
// // uint8_t servonum17 = 11;

// // uint8_t servonum4 = 5;
// // uint8_t servonum10 = 6;
// // uint8_t servonum16 = 7;

// // uint8_t servonum3 = 0;
// // uint8_t servonum9 = 1;
// // uint8_t servonum15 = 2;

// // //PWM2
// // uint8_t servonum0 = 9;
// // uint8_t servonum6 = 10;
// // uint8_t servonum12 = 11;

// // uint8_t servonum1 = 5;
// // uint8_t servonum7 = 6;
// // uint8_t servonum13 = 7;

// // uint8_t servonum2 = 0;
// // uint8_t servonum8 = 1;
// // uint8_t servonum14 = 2;



// // //PWM1
// #define servonum5 9
// #define servonum11 10
// #define servonum17 11

// #define servonum4 5
// #define servonum10 6
// #define servonum16 7

// #define servonum3 0
// #define servonum9 1
// #define servonum15 2

// // //PWM2
// #define servonum0 9
// #define servonum6 10
// #define servonum12 11

// #define servonum1 5
// #define servonum7 6
// #define servonum13 7

// #define servonum2 0
// #define servonum8 1
// #define servonum14 2

#define SPEED 200 // Distance robot moves in mm, can be adjusted as needed
#define INTERPOLATION_SIZE 20 // Number of interpolation points, can be adjusted as needed
#define SPACING (SPEED / INTERPOLATION_SIZE) // There will be SPEED / SPACING number of interpolation points