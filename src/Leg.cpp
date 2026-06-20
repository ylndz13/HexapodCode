#include <Arduino.h>
#include "Leg.h"
#include "math.h"

float Leg::toDegrees(float rad) {
    return rad * (180.0f / 3.14159265358979323846f);
}

float Leg::toRadians(float deg) {
    return deg * (3.14159265358979323846f / 180.0f);
}

void Leg::initialize() {
    moveCoxa(servo3.midVal - coxaAng); // angle is how much more to move the servo from its current position
    moveFemur(30);
    moveTibia(servo1.midVal - tibiaAng - 45);
}

void Leg::interpolate(int targetX, int targetY, int targetZ) {
    float ratio = 1.0f / ((SPEED / SPACING) / 2);
    float l = sqrt(targetX * targetX + targetY * targetY + targetZ * targetZ); // length of the line segment
    float t;
    float tP;

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

    // last interpolated point is at (0, 0, 0)
    interpolation[INTERPOLATION_SIZE].x = 0;
    interpolation[INTERPOLATION_SIZE].y = 0;
    interpolation[INTERPOLATION_SIZE].z = 0;
}

void Leg::calculateIK(const Vector3& current, const Vector3& target) {
    float legLength = l1 + l2 * sin(toRadians(currFemurAng)) + l3 * sin(toRadians(-165.96 + currFemurAng + currTibiaAng));

    // delZ is the difference in the z coord of the foot and middle of the coxa 
    float delZ = cos(toRadians(-165.96 + currFemurAng + currTibiaAng)) * l3 - cos(toRadians(180 - currFemurAng)) * l2 - (target.z - current.z);

    // Assign values to footLoc based on leg length
    footLoc.x = legLength * cos(toRadians(coxaAng - servo3.midVal));
    if (servo3.pwmNum == 1) {
        footLoc.y = legLength * sin(-toRadians(coxaAng - servo3.midVal));
    } else if (servo3.pwmNum == 2) {
        footLoc.y = legLength * sin(toRadians(coxaAng - servo3.midVal));
    } else {
        Serial.println("Invalid pwmNum for coxa servo");
        return;
    }
    footLoc.z = delZ;

    // projectedLength on the xy plane, independent of z
    float projectedLength = (target - current + footLoc).xyProj();

    // Calculates theta relative to the x axis
    theta = toDegrees(atan2(target.y, projectedLength));

    // Calculates alpha relative to the perpendicular of the femur servo
    alpha = toDegrees(acosf((- l3 * l3 + l2 * l2 + (projectedLength - l1) * (projectedLength - l1) + delZ * delZ) 
        / (2 * sqrt(delZ * delZ + (projectedLength - l1) * (projectedLength - l1)) * l2)) - atan2(delZ, projectedLength - l1));
        
    // Calculates beta relative to the perpendicular of the tibia servo
    beta = toDegrees(acosf((l3 * l3 + l2 * l2 - (projectedLength - l1) * (projectedLength - l1) - delZ * delZ) 
        / (2 * l3 * l2)));        
}

void Leg::moveCoxa(float angle) {
        if (servo3.pwmNum == 2 && (servo3.midVal + angle) <= 180 && (servo3.midVal + angle) >= 0) { // Left side
            
            pwm2.setPWM(servo3.servoNum, 0, map(servo3.midVal + angle, 0, 180, servo3.minPwm, servo3.maxPwm));
            
            currCoxaAng += servo3.midVal + angle - coxaAng; // Update the current coxa angle
            // Serial.print("currCoxaAng: "); Serial.println(currCoxaAng);
            
            coxaAng = servo3.midVal + angle; // Update the current coxa angle
            // Serial.print("Moving coxa servo for pwm 2 to angle: "); Serial.println(servo3.midVal + angle);

        } else if (servo3.pwmNum == 1 && (servo3.midVal - angle) <= 180 && (servo3.midVal - angle) >= 0) { // Right side
            
            pwm1.setPWM(servo3.servoNum, 0, map(servo3.midVal - angle, 0, 180, servo3.minPwm, servo3.maxPwm));
            
            currCoxaAng += servo3.midVal - angle - coxaAng; // Update the current coxa angle
            // Serial.print("currCoxaAng: "); Serial.println(currCoxaAng);

            coxaAng = servo3.midVal - angle; // Update the current coxa angle
            // Serial.print("Moving coxa servo for pwm 1 to angle: "); Serial.println(servo3.midVal - angle);

        } else {
            Serial.println("Invalid angle or config for coxa servo");
        }
}

void Leg::moveFemur(float angle) {
    if (servo2.pwmNum == 2 && (servo2.midVal + angle) <= 180 && (servo2.midVal + angle) >= 0) { // Left side

        currFemurAng = 90 + angle - 8.787; // Update the current femur angle
        // Serial.print("currFemurAng: "); Serial.println(currFemurAng);

        femurAng = servo2.midVal + angle; // Update the current femur angle
        // Serial.print("Moving femur servo for pwm 2 to angle: "); Serial.println(femurAng);

        pwm2.setPWM(servo2.servoNum, 0, map(femurAng, 0, 180, servo2.minPwm, servo2.maxPwm));

    } else if (servo2.pwmNum == 1 && (servo2.midVal + angle) <= 180 && (servo2.midVal + angle) >= 0) { // Right side

        currFemurAng = 90 + angle - 8.787; // Update the current femur angle
        // Serial.print("currFemurAng: "); Serial.println(currFemurAng);

        femurAng = servo2.midVal + angle; // Update the current femur angle
        // Serial.print("Moving femur servo for pwm 1 to angle: "); Serial.println(femurAng);

        pwm1.setPWM(servo2.servoNum, 0, map(femurAng, 0, 180, servo2.minPwm, servo2.maxPwm));

    } else {
        Serial.println("Invalid angle or config for femur servo");
    }
}

void Leg::moveTibia(float angle) {
    if (servo1.pwmNum == 2 && (tibiaAng + angle) <= 180 && (tibiaAng + angle) >= 0) { // Left side
            
        pwm2.setPWM(servo1.servoNum, 0, map(tibiaAng + angle, 0, 180, servo1.minPwm, servo1.maxPwm));
    
        tibiaAng = tibiaAng + angle; // Update the current tibia angle
            
        currTibiaAng = currTibiaAng + angle; // Update the current tibia angle
        // Serial.print("Moving tibia servo for pwm 2 to angle: "); Serial.println(tibiaAng);

    } else if (servo1.pwmNum == 1 && (tibiaAng + angle) <= 180 && (tibiaAng + angle) >= 0) { // Right side
            
        pwm1.setPWM(servo1.servoNum, 0, map(tibiaAng + angle, 0, 180, servo1.minPwm, servo1.maxPwm));
            
        tibiaAng = tibiaAng + angle; // Update the current tibia angle
            
        currTibiaAng = currTibiaAng + angle; // Update the current tibia angle
        // Serial.print("Moving tibia servo for pwm 1 to angle: ");  Serial.println(tibiaAng);

    } else {
        Serial.println("Invalid angle or config for tibia servo");
    }
}