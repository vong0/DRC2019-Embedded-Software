#ifndef ROBOT_BASE_H
#define ROBOT_BASE_H

#include "Motor.h"
#include "myBNO080.h"
#include "Config.h"
#include "Pose.h"

class RobotBase {
private:
    // Motors
    Motor leftMotor;
    Motor rightMotor;
    myBNO080 imu;

    // Update status - set flags when data available
    bool updateStatus[3] = {0};     // [leftMotor, rightMotor, imu] 

    // Speed actual/desired
    float v_cm, v_d;    // velocity of centre of wheels
    float w_cm, w_d;    // angular velocity of centre

public:
    // Config
    RobotBase(uint32_t T = SAMPLE_TIME, uint32_t resetTime = RESET_TIME);
    void begin();

    // Drive - call in this order
    void setSpeed(float v, float w);
    void setSpeed(const ControlInput& input);
    bool sampleData();
    void resetFlags();
    void drive();

    // Getters
    float getVcm();             // m/s
    float getWcm();             // rad/s
    float getYaw();             // rad
    float getVd();              // m/s
    float getWd();              // rad/s
    float getDeltaTime();       // s

    // Debug
    float getLeftSpeed();           // m/s
    float getRightSpeed();          // m/s
    float getDesiredLeftSpeed();    // m/s
    float getDesiredRightSpeed();   // m/s
    float getYawDeg();              // deg
};


#endif
