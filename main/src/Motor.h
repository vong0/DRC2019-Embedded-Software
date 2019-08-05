#ifndef MOTOR_H
#define MOTOR_H

#include "MotorBase.h"
#include "Config.h"

// Identical to MotorBase. Specify m/s instead of rad/s
class Motor: public MotorBase {
public:
    // Config
    Motor(bool x, uint32_t T);   // which motor? LEFT_MOTOR or RIGHT_MOTOR
    void begin();

    // Setters
    void setSpeed(float v);

    // Getters
    float getDesiredSpeed();
    float getSpeed();
    float getError();

    // Helper functions
    float saturate(const float& x, const float& min, const float& max);
};

#endif