#include "Motor.h"

// Config
Motor::Motor(bool x, uint32_t T) 
    : MotorBase(x, T) {
}

// Initialise and set up gains
void Motor::begin() {
    MotorBase::begin();
    setGains(motor_e_gains, motor_N_e, motor_m_gains, motor_N_m);
}

// Setters
void Motor::setSpeed(float v) {
    float w_d = saturate(v/WHEEL_RADIUS, -MAX_RADIAN_SPEED, MAX_RADIAN_SPEED);
    setRadianSpeed(w_d);
}

// Getters
float Motor::getDesiredSpeed() {
    return getDesiredRadianSpeed()*WHEEL_RADIUS;
}

float Motor::getSpeed() {
    return getRadianSpeed()*WHEEL_RADIUS;
}

float Motor::getError() {
    return (getDesiredSpeed() - getSpeed());
}

// Helper functions
float Motor::saturate(const float& x, const float& min, const float& max) {
    float y = x;
    if(y < min) {
        y = min;
    }
    else if(y > max) {
        y = max;
    }
    return y;
}