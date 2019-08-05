#include "RobotBase.h"

// Config
RobotBase::RobotBase(uint32_t T, uint32_t resetTime)
    : leftMotor(LEFT_MOTOR, T), rightMotor(RIGHT_MOTOR, T),
        imu(T/1e3, resetTime/1e3) {
}

// Initialise robot
void RobotBase::begin() {
    leftMotor.begin();
    rightMotor.begin();
    imu.begin();
}

// Sets desired v and w
void RobotBase::setSpeed(float v, float w) {
    // Save desired speeds
    v_d = v;
    w_d = w;

    // Calculate individual wheel speeds
    float tmp = w*WHEEL_DIST/2;
    float v_L = v_d - tmp;
    float v_R = v_d + tmp;

    // Prioritise angular speed over linear speed
    if(v_L > MAX_WHEEL_SPEED && v_R > MAX_WHEEL_SPEED) {
        float reduceAmount = max( abs(v_L), abs(v_R) ) - MAX_WHEEL_SPEED;
        v_d -= reduceAmount;
        v_L -= reduceAmount; v_R -= reduceAmount;
    }

    // Set motor speeds
    leftMotor.setSpeed(v_L);
    rightMotor.setSpeed(v_R);
}

// Sets desired v and w using ControlInput struct
void RobotBase::setSpeed(const ControlInput& input) {
    setSpeed(input.v, input.w);
}

// Update data for robot
// Returns true if data was updated
bool RobotBase::sampleData() {
    // Update data for motors
    updateStatus[0] |= leftMotor.sampleData();
    updateStatus[1] |= rightMotor.sampleData();

    //// Update only once - imu uses polling that affects T
    if(!updateStatus[2]) {
        updateStatus[2] |= imu.update();
    }

    // Update robot data only if all data received
    if(updateStatus[0] && updateStatus[1] && updateStatus[2]) {
        v_cm = (rightMotor.getSpeed() + leftMotor.getSpeed())/2;
        w_cm = (rightMotor.getSpeed() - leftMotor.getSpeed())/WHEEL_DIST;

        // Reset flags
        resetFlags();
        return true;
    }
    
    return false;
}

// Reset update flags
void RobotBase::resetFlags() {
    updateStatus[0] = false;
    updateStatus[1] = false;
    updateStatus[2] = false;
}

// Drive robot
void RobotBase::drive() {
    // Control speed
    leftMotor.controlSpeed();
    rightMotor.controlSpeed();
}

// Get velocity
float RobotBase::getVcm() {
    return v_cm;
}

// Get anguar velocity
float RobotBase::getWcm() {
    return w_cm;
}

// Gets yaw (since IMU IS UPSIDE DOWN)
float RobotBase::getYaw() {
    return imu.getYaw();
}

// Get desired velocity
float RobotBase::getVd() {
    return v_d;
}

// Get desired anguar velocity
float RobotBase::getWd() {
    return w_d;
}

// Get sample time
float RobotBase::getDeltaTime() {
    return leftMotor.getDeltaTime(); // use either one
}

// Debug
float RobotBase::getLeftSpeed() {
    return leftMotor.getSpeed();
}

float RobotBase::getRightSpeed() {
    return rightMotor.getSpeed();
}

float RobotBase::getDesiredLeftSpeed() {
    return leftMotor.getDesiredSpeed();
}

float RobotBase::getDesiredRightSpeed() {
    return rightMotor.getDesiredSpeed();
}

// Gets yaw
float RobotBase::getYawDeg() {
    return imu.getYawDeg();
}
