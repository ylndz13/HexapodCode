#include <Arduino.h>
#include "Robot.h"

Robot::Robot(int x, int y, int z) {
    currCoord.x = x;
    currCoord.y = y;
    currCoord.z = z;

    Serial.print("Robot initialized: ");
    Serial.print(currCoord.x); Serial.print(", ");
    Serial.print(currCoord.y); Serial.print(", ");
    Serial.println(currCoord.z);
}

void Robot::updateCurrCoord(const Vector3& target) {
    currCoord.x += target.x;
    currCoord.y += target.y;
    currCoord.z += target.z;

    Serial.println("Updated current coordinates: ");
    Serial.print("currXCoord: "); Serial.print(currCoord.x);
    Serial.print(", currYCoord: "); Serial.print(currCoord.y);
    Serial.print(", currZCoord: "); Serial.println(currCoord.z);
}
void Robot::tripodMoveServos(Leg& legA, Leg& legB, Leg& legC) {
        
    // -14.04 is the offset for calculating the tibia angle: legC.beta - (legC.currTibiaAng - 38.799) - 52.799
    legA.moveCoxa(legA.theta); legA.moveFemur(legA.alpha + 8.787); legA.moveTibia(legA.beta - legA.currTibiaAng - 14.04);
    legB.moveCoxa(legB.theta); legB.moveFemur(legB.alpha + 8.787); legB.moveTibia(legB.beta - legB.currTibiaAng - 14.04);        
    legC.moveCoxa(legC.theta); legC.moveFemur(legC.alpha + 8.787); legC.moveTibia(legC.beta - legC.currTibiaAng - 14.04);
    delay(75);
}

void Robot::tripodWalk(Leg& legA, Leg& legB, Leg& legC, const Vector3& target) { // tibias 0, 2, and 4
                // Need to call the legs by reference (&) to prevent memory corruption. Calling
                // just the values of Leg causes undefined behavior.
        
    // TODO 1: adjust leg angles (especially leg 4 s.t. each leg fully leaves the ground during traversal)
    // TODO 2: figure out why repeated calls of A and D give invalid femur and tibia angles
    // TODO 2.5: investigate the possibility of adding rpi + vision camera + april tags / targets, swtich for power system
    // print spare plates for femur servo mount (the one with bearing)
    // TODO 3: if time allows, work on the 4 modes
    if (target.x == 0) {
        // y and z direction displacements only
        legA.interpolate(target.x, target.y, target.z);
        legB.interpolate(target.x, target.y, target.z);
        legC.interpolate(target.x, target.y, target.z);
    } else {
        // Negate the x coordinate for the legs based on their pwmNum for side to side movement.
        int factorA = (legA.servo1.pwmNum == 1) ? 1 : -1;
        int factorB = (legB.servo1.pwmNum == 1) ? 1 : -1;
        int factorC = (legC.servo1.pwmNum == 1) ? 1 : -1;

        legA.interpolate(factorA * target.x, target.y, target.z);
        legB.interpolate(factorB * target.x, target.y, target.z);
        legC.interpolate(factorC * target.x, target.y, target.z);
    }

    // Calculate IK based on interpolated points
    for (int i = 0; i < (INTERPOLATION_SIZE + 1); i++) {
        legA.calculateIK(((i > 0) ? legA.interpolation[i - 1] : Vector3(0,0,0)), legA.interpolation[i]);
        legB.calculateIK(((i > 0) ? legB.interpolation[i - 1] : Vector3(0,0,0)), legB.interpolation[i]);         
        legC.calculateIK(((i > 0) ? legC.interpolation[i - 1] : Vector3(0,0,0)), legC.interpolation[i]);

        // Move the servos of the legs
        tripodMoveServos(legA, legB, legC);
    }
}

void Robot::robotMove(const Vector3& target) {
    tripodWalk(leg0, leg2, leg4, target / 2); // left walk
    updateCurrCoord(target / 2);
    delay(100);

    tripodWalk(leg1, leg3, leg5, target / 2); // right walk
    delay(100);
    updateCurrCoord(target / 2);
}