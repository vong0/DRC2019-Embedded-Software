#ifndef MOTOR_BASE_H
#define MOTOR_BASE_H

#include "Arduino.h"
#include "SpeedEncoder.h"
#include "MotorPWM.h"
#include "Controller.h"
#include "Config.h"

// PWM settings
#define PWM_PERIOD 3000         // microseconds
#define PWM_PULSE_LOW 1000      // microseconds
#define PWM_PULSE_HIGH 2000     // microseconds  

// Control range
#define CONTROL_MIN -1024
#define CONTROL_MAX 1024

// Wheel parameters
#define TICK_RATIO 1920

// Motor selection
#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1

class MotorBase {
private:
    SpeedEncoder enc;       // encoder
    MotorPWM pwm;           // pwm of the motor
    Controller cont;        // controller

    // Speed variables - ease of use
    float w_d;              // desired speed (rad/s)
    float w_a;              // actual speed (rad/s)
    float e;                // error in speed (rad/s)
    float m;                // control effort (control range)
public:
    // Config
    MotorBase(bool x, uint32_t T);
    void begin();

    // Controller
    void setGains(float *e_in, uint8_t _N_e, float *m_in, uint8_t _N_m);
    void setRadianSpeed(float w);   // set speed BEFORE control
    void controlSpeed();            // writes to PWM based on control effort

    // Encoder
    bool sampleData();              // updates all variables
    float getTickSpeed();           // counts/s
    uint32_t getCurrTime();    // microseconds
    float getDeltaTime();           // s

    // Motor variables
    float getDesiredRadianSpeed();      // rad/s
    float getRadianSpeed();             // rad/s
    float getRadianError();             // rad/s
    float getControlEffort();           // control range
    float getControlEffort(float e);    // control range

    // Print configs
    void printSettings();

    // PWM direct control
    void writeValue(float value);
    uint16_t getPWMOutput();

    // Debug encoder variables
    int32_t getCurrTicks();                // count/s
    int32_t getPrevTicks();                // counts/s
    int32_t getDeltaTicks();               // counts/s
};


#endif