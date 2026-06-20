#include "Vector3.h"
#include "Constants.h"
#include "Devices.h"

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
        float currCoxa, float currFemur, float currTibia);

    // Convert radians to degrees
    float toDegrees(float rad);

    // Convert degrees to radians
    float toRadians(float deg);

    /* Initializes the robot's coordinate
    */
    void initialize();

    /* Interpolates (x, y, and z) based on the curve using Bezier Curve with one
    * control point at x = midpoint of moving distance and y = 0.3 * x. The x and y coordinates are
    * calculated by scaling the targetX and targetY coordinates by t.
    * Bezier curve found here: https://www.desmos.com/calculator/cahqdxeshd.
    */
    void interpolate(int targetX, int targetY, int targetZ);

    /* Calculates the angles alpha, beta, and theta based on the x, y, z coordinates
    * Reference: https://www.alanzucconi.com/2020/09/14/inverse-kinematics-in-3d/.
    * delta is the difference between the default foot position and the target position
    */ 
    void calculateIK(const Vector3& current, const Vector3& target);
        
    /* Move the coxa servo (servo3) to the specified angle. Side determined from the pwmNum of the servoConfig.
    * Servo number is implied by the joint type, so no need to pass it in.
    */
    void moveCoxa(float angle);

    void moveFemur(float angle);

    void moveTibia(float angle);
    
};