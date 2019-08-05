#ifndef MOTORPWM_H
#define MOTORPWM_H

#include "Arduino.h"
#include "PWM.h"

// NOTE: PWM maps from 0-2^16-1 = 0-65535 (16-bits)
// DO NOT use timer0 pins since it is linked to micros(), millis()
// Array to show if timer has already been initialised

#define PWM_MAX_DUTY 65535

class MotorPWM {
private:
    // Hardware settings
    uint8_t pin;                // PWM pin
    uint32_t period;            // PWM input period (microseconds)

    // Define range limits
    uint32_t t_min, t_max;      // PWM input pulse range (microseconds)
    int16_t in_min, in_max;     // user defined range
    uint16_t out_min, out_max;  // 0-65535 PWM units (based on t_min, t_max)

    // PWM data
    uint16_t pwm_out;            // pwm output for debug   

    // Array of timers init
    static bool ATMEGA_timers[5];
public:
    // Config hardware
    MotorPWM(uint8_t _pin);
    
    // PWM settings
    void setInputPulse(uint32_t _period, uint32_t _t_min, uint32_t _t_max);
    void setRange(int16_t _in_min, int16_t _in_max);
    bool begin();

    // PWM control
    void writeValue(float value);    // Write PWM using custom range
    uint16_t getPWMOutput();

    // Helper function
    uint16_t mapFloatPWM(float value);

    // Print config
    void printSettings();
};

#endif