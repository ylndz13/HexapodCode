#include "Vector3.h"
#include "Leg.h"

class Robot {
    public:

    Vector3 currCoord;

    Robot(int x, int y, int z);

    void updateCurrCoord(const Vector3& target);

    /* Moves the servos of the robot in a tripod walk configuration.
    *  Precondition: legA's servo label < legB's servo label < legC's servo label.
    */
    void tripodMoveServos(Leg& legA, Leg& legB, Leg& legC);

    /* Moves the robot by some distance in the x and y directions.
    *  Precondition: legA's servo label < legB's servo label < legC's servo label.
    */
    void tripodWalk(Leg& legA, Leg& legB, Leg& legC, const Vector3& target);

    void robotMove(const Vector3& target);

    void defaultPosition();


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

    // Params: servoID, servoMin, servoMax, pwm1 or pwm2, middle angle
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
};
