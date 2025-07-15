#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include "Constants.cpp"
#include "Vector3.h"

/**
 * This file implements methods controlling the movement of the robot.
 * It includes functions to move the robot forward, backward, and to turn it left or right.
 * The movement is controlled by setting the speed of the motors and adjusting the direction.
 * The robot can also stop its movement.
 */

extern Adafruit_PWMServoDriver pwm1;
extern Adafruit_PWMServoDriver pwm2;

struct servoConfig {
    uint8_t servoNum; // The servo number on the PCA9685 board
    int minPwm; // The minimum pwm for the servo
    int maxPwm; // The maximum pwm for the servo
    int pwmNum; // location of the PCA9685 board: 1 or 2. 1 is left side, 2 is right side
    float midVal;

    void move(float angle) {
        // Map the angle to the PWM range
        int pwmValue = map(angle, 0, 180, minPwm, maxPwm);
        // Set the PWM value for the servo
        if (pwmNum == 1) {
            pwm1.setPWM(servoNum, 0, pwmValue);
        } else {
            pwm2.setPWM(servoNum, 0, pwmValue);
        }
    }
    servoConfig(uint8_t num, int minPwm, int maxPwm, int pwmBoard, float midVal)
        : servoNum(num), minPwm(minPwm), maxPwm(maxPwm), pwmNum(pwmBoard), midVal(midVal) {}
};

class Leg {
    public:

    #pragma region "Leg Setups"
    // servo numbers correspond to numbers on servo holder, not the PCA pins
    servoConfig servo1; // tibia servo
    servoConfig servo2; // femur servo
    servoConfig servo3; // coxa servo
    
    // lengths of the leg segments in mm
    int l1; // coxa length
    int l2; // femur length
    int l3; // tibia length
    
    // angles of the servos in degrees
    float coxaAng; // angle of the coxa servo relative to the body
    float femurAng; // angle of the femur servo relative to the coxa
    float tibiaAng; // angle of the tibia servo relative to the femur

    float currCoxaAng; // angle of the coxa relative to the body in degrees
    float currFemurAng; // angle of the femur relative to the coxa in degrees (horizontal line to l2))
    float currTibiaAng; // angle of the tibia relative to the (l2 to l3)

    float theta = 0; // coxa angle in degrees
    float alpha = 0; // femur angle
    float beta = 0; // tibia angle

    Vector3 footLoc;

    Vector3 interpolation[INTERPOLATION_SIZE + 1];  // The coordinates of the interpolated points. The x and y coordinates
                                     // can be scaled using the variable t from the interpolation method

    Leg(servoConfig s1, servoConfig s2, servoConfig s3, int length1, int length2, int length3, float t, float a, float b, 
        float currCoxa, float currFemur, float currTibia) : servo1(s1), servo2(s2), servo3(s3), 
        l1(length1), l2(length2), l3(length3), coxaAng(t), femurAng(a), tibiaAng(b), 
        currCoxaAng(currCoxa), currFemurAng(currFemur), currTibiaAng(currTibia) {}

    float toDegrees(float rad) {
        return rad * (180.0f / 3.14159265358979323846f); // Convert radians to degrees
    }

    float toRadians(float deg) {
        return deg * (3.14159265358979323846f / 180.0f); // Convert degrees to radians
    }

    void initialize() {
        moveCoxa(0); // angle is how much more to move the servo from its current position
        moveFemur(30);
        moveTibia(0);
    }
    #pragma endregion

    /* Interpolates (x, y, and z) based on the curve using Bezier Curve with one
    * control point at x = midpoint of moving distance and y = 0.3 * x. The x and y coordinates are
    * calculated by scaling the targetX and targetY coordinates by t.
    * Bezier curve found here: https://www.desmos.com/calculator/cahqdxeshd.
    */
    void interpolate(int targetX, int targetY, int targetZ) { // adds the interpolated points 
        // TODO 1: add SERVO_SPEED to control the number of interpolation points so we have ~5mm for each interpolated step size
        // TODO 2: add one more element at end of the interpolation array s.t. end interpolated point reaches 0,0,0
        float ratio = 1.0f / ((SPEED / SPACING) / 2);
        float l = sqrt(targetX * targetX + targetY * targetY + targetZ * targetZ); // length of the line segment
        float t;
        float tP;
        Serial.println("Interpolating");

        for (int i = 0; i < (SPEED / 2 / SPACING + 1); i++) {
            t = i * ratio;
            tP = 1 - t; // tP is t prime (1-t)
            
            interpolation[i].x = t * targetX;
            
            interpolation[i].y = t * targetY;

            interpolation[i].z = (tP * (tP * (t * l * 0.3f) + t * (tP * l * 0.3f + t * l * 0.3f)) 
                 + t * (tP * (tP * l * 0.3f + t * l * 0.3f) + t * (tP * l * 0.3f + t * targetZ)));
        }

        for (int j = (INTERPOLATION_SIZE / 2); j < INTERPOLATION_SIZE; j++) {
            interpolation[j].x = (INTERPOLATION_SIZE - j) * ratio * targetX;

            interpolation[j].y = (INTERPOLATION_SIZE - j) * ratio * targetY;

            interpolation[j].z = 0;
        }

        interpolation[INTERPOLATION_SIZE].x = 0; // last point is at (0, 0, 0)
        interpolation[INTERPOLATION_SIZE].y = 0;
        interpolation[INTERPOLATION_SIZE].z = 0;
    }

    /* Calculates the angles alpha, beta, and theta based on the x, y, z coordinates
    * Reference: https://www.alanzucconi.com/2020/09/14/inverse-kinematics-in-3d/.
    * delta is the difference between the default foot position and the target position
    */ 
    void calculateIK(const Vector3& current, const Vector3& target) {
        
        float legLength = l1 + l2 * sin(toRadians(currFemurAng)) + l3 * sin(toRadians(-165.96 + currFemurAng + currTibiaAng));

        // delZ is the difference in the z coord of the foot and middle of the coxa 
        float delZ = cos(toRadians(-165.96 + currFemurAng + currTibiaAng)) * l3 - cos(toRadians(180 - currFemurAng)) * l2 - (target.z - current.z);
        Serial.print("*****delZ: "); Serial.println(delZ);

        // Assign values to footLoc based on leg length
        footLoc.x = legLength * cos(toRadians(coxaAng - servo3.midVal));
        footLoc.y = legLength * sin(toRadians(coxaAng - servo3.midVal));
        footLoc.z = delZ;
        Serial.print("*****footLoc: "); footLoc.print();

        // projectedLength on the xy plane, independent of z
        float projectedLength = (target - current + footLoc).xyProj();
        Serial.print("*****projected length: "); Serial.println(projectedLength);

        // Calculates theta relative to the x axis
        theta = toDegrees(atan2(target.y, projectedLength));

        // Calculates alpha relative to the perpendicular of the femur servo
        alpha = toDegrees(acosf((- l3 * l3 + l2 * l2 + (projectedLength - l1) * (projectedLength - l1) + delZ * delZ) 
            / (2 * sqrt(delZ * delZ + (projectedLength - l1) * (projectedLength - l1)) * l2)) - atan2(delZ, projectedLength - l1));
        
        // Calculates beta relative to the perpendicular of the tibia servo
        beta = toDegrees(acosf((l3 * l3 + l2 * l2 - (projectedLength - l1) * (projectedLength - l1) - delZ * delZ) 
            / (2 * l3 * l2)));
        
        Serial.print("Calculated angles: theta="); Serial.print(theta);
        Serial.print(", alpha="); Serial.print(alpha);
        Serial.print(", beta="); Serial.println(beta);
    }
        
    /* Move the coxa servo (servo3) to the specified angle. Side determined from the pwmNum of the servoConfig.
    * Servo number is implied by the joint type, so no need to pass it in.
    */
    void moveCoxa(float angle) {

        if (servo3.pwmNum == 2 && (servo3.midVal + angle) <= 180 && (servo3.midVal + angle) >= 0) { // Left side
            
            pwm2.setPWM(servo3.servoNum, 0, map(servo3.midVal + angle, 0, 180, servo3.minPwm, servo3.maxPwm));
            
            currCoxaAng += servo3.midVal + angle - coxaAng; // Update the current coxa angle
            // Serial.print("currCoxaAng: "); Serial.println(currCoxaAng);
            
            coxaAng = servo3.midVal + angle; // Update the current coxa angle
            Serial.print("Moving coxa servo for pwm 2 to angle: "); Serial.println(servo3.midVal + angle);

        } else if (servo3.pwmNum == 1 && (servo3.midVal - angle) <= 180 && (servo3.midVal - angle) >= 0) { // Right side
            
            pwm1.setPWM(servo3.servoNum, 0, map(servo3.midVal - angle, 0, 180, servo3.minPwm, servo3.maxPwm));
            
            currCoxaAng += servo3.midVal - angle - coxaAng; // Update the current coxa angle
            // Serial.print("currCoxaAng: "); Serial.println(currCoxaAng);

            coxaAng = servo3.midVal - angle; // Update the current coxa angle
            Serial.print("Moving coxa servo for pwm 1 to angle: "); Serial.println(servo3.midVal - angle);

        } else {
            Serial.println("Invalid angle or config for coxa servo");
        }
    }

    void moveFemur(float angle) {

        if (servo2.pwmNum == 2 && (servo2.midVal + angle) <= 180 && (servo2.midVal + angle) >= 0) { // Left side

            currFemurAng = 90 + angle - 8.787; // Update the current femur angle
            Serial.print("currFemurAng: "); Serial.println(currFemurAng);

            femurAng = servo2.midVal + angle; // Update the current femur angle
            Serial.print("Moving femur servo for pwm 2 to angle: "); Serial.println(femurAng);

            pwm2.setPWM(servo2.servoNum, 0, map(femurAng, 0, 180, servo2.minPwm, servo2.maxPwm));

        } else if (servo2.pwmNum == 1 && (servo2.midVal + angle) <= 180 && (servo2.midVal + angle) >= 0) { // Right side

            currFemurAng = 90 + angle - 8.787; // Update the current femur angle
            Serial.print("currFemurAng: "); Serial.println(currFemurAng);

            femurAng = servo2.midVal + angle; // Update the current femur angle
            Serial.print("Moving femur servo for pwm 1 to angle: "); Serial.println(femurAng);

            pwm1.setPWM(servo2.servoNum, 0, map(femurAng, 0, 180, servo2.minPwm, servo2.maxPwm));

        } else {
            Serial.println("Invalid angle or config for femur servo");
        }
    }

    void moveTibia(float angle) {
        if (servo1.pwmNum == 2 && (tibiaAng + angle) <= 180 && (tibiaAng + angle) >= 0) { // Left side
            
            pwm2.setPWM(servo1.servoNum, 0, map(tibiaAng + angle, 0, 180, servo1.minPwm, servo1.maxPwm));
            
            tibiaAng = tibiaAng + angle; // Update the current tibia angle
            
            currTibiaAng = currTibiaAng + angle; // Update the current tibia angle
            Serial.print("Moving tibia servo for pwm 2 to angle: "); Serial.println(tibiaAng);

        } else if (servo1.pwmNum == 1 && (tibiaAng + angle) <= 180 && (tibiaAng + angle) >= 0) { // Right side
            
            pwm1.setPWM(servo1.servoNum, 0, map(tibiaAng + angle, 0, 180, servo1.minPwm, servo1.maxPwm));
            
            tibiaAng = tibiaAng + angle; // Update the current tibia angle
            
            currTibiaAng = currTibiaAng + angle; // Update the current tibia angle
            Serial.print("Moving tibia servo for pwm 1 to angle: ");  Serial.println(tibiaAng);

        } else {
            Serial.println("Invalid angle or config for tibia servo");
        }
    }
};

class Robot {
    public:

    #pragma region "Servo Configurations"
    // servo setups, 1 is right 2 is left
    servoConfig servo5 = {9, 150, 610, 1, 90.0f};
    servoConfig servo11 = {10, 130, 600, 1, 90.0f};
    servoConfig servo17 = {11, 120, 580, 1, 90.0f};

    servoConfig servo4 = {5, 120, 600, 1, 100.0f};
    servoConfig servo10 = {6, 180, 640, 1, 90.0f};
    servoConfig servo16 = {7, 110, 510, 1, 100.0f};

    servoConfig servo3 = {0, 200, 600, 1, 90.0f};
    servoConfig servo9 = {1, 150, 620, 1, 90.0f};
    servoConfig servo15 = {2, 140, 600, 1, 90.0f};

    servoConfig servo0 = {9, 150, 700, 2, 90.0f};
    servoConfig servo6 = {10, 120, 620, 2, 90.0f};
    servoConfig servo12 = {11, 110, 630, 2, 85.0f};

    servoConfig servo1 = {5, 130, 700, 2, 95.0f};
    servoConfig servo7 = {6, 150, 670, 2, 95.0f};
    servoConfig servo13 = {7, 150, 600, 2, 95.0f};

    servoConfig servo2 = {0, 150, 700, 2, 90.0f};
    servoConfig servo8 = {1, 130, 650, 2, 87.0f};
    servoConfig servo14 = {2, 150, 660, 2, 90.0f};

    // Params: servo1, 2, 3, coxa, femur, tibia length, C's, F's, T's servo angle, C,F,T's physical angle 
    Leg leg0 = {servo0, servo6, servo12, 62, 83, 112, 85.0f, 120.0f, 45.0f, 90.0f, 111.213f, 60.689f}; // left, pwm 2
    Leg leg1 = {servo1, servo7, servo13, 62, 83, 112, 95.0f, 125.0f, 50.0f, 90.0f, 111.213f, 60.689f}; // left, pwm 2
    Leg leg2 = {servo2, servo8, servo14, 62, 83, 112, 90.0f, 117.0f, 45.0f, 90.0f, 111.213f, 60.689f}; // left, pwm 2
    Leg leg3 = {servo3, servo9, servo15, 62, 83, 112, 90.0f, 120.0f, 45.0f, 90.0f, 111.213f, 60.689f}; // right, pwm 1
    Leg leg4 = {servo4, servo10, servo16, 62, 83, 112, 100.0f, 120.0f, 55.0f, 90.0f, 111.213f, 60.689f}; // right, pwm 1
    Leg leg5 = {servo5, servo11, servo17, 62, 83, 112, 90.0f, 120.0f, 45.0f, 90.0f, 111.213f, 60.689f}; // right, pwm 1
    
    Vector3 currCoord;
    
    #pragma endregion

    Robot(int x, int y, int z) {
        currCoord.x = x;
        currCoord.y = y;
        currCoord.z = z;

        Serial.print("Robot initialized: ");
        Serial.print(currCoord.x); Serial.print(", ");
        Serial.print(currCoord.y); Serial.print(", ");
        Serial.println(currCoord.z);
    }

    void updateCurrCoord(const Vector3& target) {
        currCoord.x += target.x;
        currCoord.y += target.y;
        currCoord.z += target.z;

        Serial.println("Updated current coordinates: ");
        Serial.print("currXCoord: "); Serial.print(currCoord.x);
        Serial.print(", currYCoord: "); Serial.print(currCoord.y);
        Serial.print(", currZCoord: "); Serial.println(currCoord.z);
    }

    /* Moves the servos of the robot in a tripod walk configuration.
    *  Precondition: legA's servo label < legB's servo label < legC's servo label.
    */
    void tripodMoveServos(Leg& legA, Leg& legB, Leg& legC) {
        
        // legA.moveCoxa(legA.theta); legA.moveFemur(legA.alpha + 90 - legA.currFemurAng); legA.moveTibia(legA.beta - legA.currTibiaAng);
        // legB.moveCoxa(legB.theta); legB.moveFemur(legB.alpha + 90 - legB.currFemurAng); legB.moveTibia(legB.beta - legB.currTibiaAng);
        
        Serial.print("***** angle being called: "); Serial.println(legC.alpha);

        // -14.04 is the offset for calculating the tibia angle: legC.beta - (legC.currTibiaAng - 38.799) - 52.799
        legC.moveCoxa(legC.theta); legC.moveFemur(legC.alpha + 8.787); legC.moveTibia(legC.beta - legC.currTibiaAng - 14.04);
        // Serial.print("Moving leg C coxa: "); Serial.println(legC.theta);
        // Serial.print("Moving leg C femur: "); Serial.println(legC.alpha + 90 - legC.currFemurAng);
        // Serial.print("Moving leg C tibia: "); Serial.println(legC.beta - (legC.currTibiaAng - 38.799) - 52.799);
    }

    /* Moves the robot by some distance in the x and y directions.
    *  Precondition: legA's servo label < legB's servo label < legC's servo label.
    */
    void tripodWalk(Leg& legA, Leg& legB, Leg& legC, const Vector3& target) { // tibias 0, 2, and 4
                    // Need to call the legs by reference (&) to prevent memory corruption. Calling
                    // just the values of Leg causes undefined behavior.

        if (target.x == 0) { // y and z direction displacements only
            // Interpolate the points for the legs
            legA.interpolate(target.x, target.y, target.z);
            legB.interpolate(target.x, target.y, target.z);
            legC.interpolate(target.x, target.y, target.z);
        } else if ((target.x < 0 && legA.servo1.pwmNum == 2) || (target.x > 0 && legA.servo1.pwmNum == 1)) { 
            // left tripod walk to the left or right tripod walk to the right. negate x-coord on legB
            legA.interpolate(target.x, target.y, target.z);
            legB.interpolate(-target.x, target.y, target.z);
            legC.interpolate(target.x, target.y, target.z);
        } else if ((target.x < 0 && legA.servo1.pwmNum == 1) || (target.x > 0 && legA.servo1.pwmNum == 2)) { 
            // right tripod walk to the left or left tripod walk to the right. negate x-coord on legA and legC
            legA.interpolate(-target.x, target.y, target.z);
            legB.interpolate(-target.x, target.y, target.z);
            legC.interpolate(target.x, target.y, target.z);
            Serial.print("**** Interpolation coordinates: ");
        } else {
            Serial.println("Invalid x coordinate for tripod walk");
            Serial.println(legA.servo1.pwmNum);
            return;
        }

        // int size = SPEED; // Assuming the size is the same for all legs
        for (int i = 0; i < (INTERPOLATION_SIZE + 1); i++) {
            // Serial.println("Leg A's IK: ");
            // legA.calculateIK(currXCoord, currYCoord, currZCoord, legA.interpolationXResults[i], 
            //     legA.interpolationYResults[i], legA.interpolationZResults[i]);
            // Serial.println("Leg B's IK: ");
            // legB.calculateIK(currXCoord, currYCoord, currZCoord, legB.interpolationXResults[i], 
            //     legB.interpolationYResults[i], legB.interpolationZResults[i]);
            Serial.println("Leg C's IK: ");
            legC.calculateIK(((i > 0) ? legC.interpolation[i - 1] : Vector3(0,0,0)), legC.interpolation[i]);
            Serial.print("Interpolation x: "); Serial.println(legC.interpolation[i].x);
            Serial.print("Interpolation y: "); Serial.println(legC.interpolation[i].y);
            Serial.print("Interpolation z: "); Serial.println(legC.interpolation[i].z);
            Serial.print("Angle theta: "); Serial.println(legC.theta);
            Serial.print("Angle alpha: "); Serial.println(legC.alpha);
            Serial.print("Angle beta: "); Serial.println(legC.beta);
            tripodMoveServos(legA, legB, legC); // Move the servos of the legs
            // delay(10); // Delay to allow the servos to move
        }
    }

    void robotMove(const Vector3& target) {
        tripodWalk(leg0, leg2, leg4, target / 2); // left walk
        delay(100);
        updateCurrCoord(target / 2);

        // tripodWalk(leg1, leg3, leg5, targetX / 2, targetY / 2, targetZ / 2); // right walk
        // delay(100);
        // updateCurrCoord(targetX / 2, targetY / 2, targetZ / 2);

        // Serial.print("Current Position: X=");
        // Serial.print(currXCoord);
        // Serial.print(", Y=");
        // Serial.print(currYCoord);
        // Serial.print(", Z=");
        // Serial.println(currZCoord);
    }

    // put function definitions here:
    void defaultPosition() {
        leg0.initialize(); leg1.initialize(); leg2.initialize();
        leg3.initialize(); leg4.initialize(); leg5.initialize();
    }


    // TODO 7: implement the 4 robot modes 
    void mode1() { // 1: mode 1: can do the lethal dance, each mode lasts for 5 seconds
        Serial.println("mode 1");
        delay(5000); // Simulate the dance for 5 seconds
    }
    void mode2() { // 2: mode 2: turn around in place
        Serial.println("mode 2");
    }

    void mode3() { // 3: mode 3: jump?
        Serial.println("mode 3");
    }

    void mode4() { // 4: mode 4: do a wave (as in like titling the robot body left and right)
        Serial.println("mode 4");
    } 
};
