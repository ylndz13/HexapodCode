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

    void robotMove(const Vector3& target, Leg& leg0, Leg& leg1, Leg& leg2, Leg& leg3, Leg& leg4, Leg& leg5);

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
};
