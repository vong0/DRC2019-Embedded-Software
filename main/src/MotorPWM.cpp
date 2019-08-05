
#include "MotorPWM.h"

// History of PWM timers already init
bool MotorPWM::ATMEGA_timers[5] = {false};

// Constructor
MotorPWM::MotorPWM(uint8_t _pin)
    : pin(_pin) {
}

// Set high time limits
void MotorPWM::setInputPulse(uint32_t _period, uint32_t _t_min, uint32_t _t_max) {
    period = _period;
    t_min = _t_min;
    t_max = _t_max;
}

// Set PWM range
void MotorPWM::setRange(int16_t _in_min, int16_t _in_max) {
    in_min = _in_min;
    in_max = _in_max;
}

// Sets the frequency of the timers
// Initialises PWM based on ranges
bool MotorPWM::begin() {
    // Find timer
    uint8_t timer = digitalPinToTimer(pin);

    // Only initialise timers only once (do not init timer0)
    if(timer > 0 && !ATMEGA_timers[timer]) {
        InitTimerByPin(pin);
        ATMEGA_timers[timer] = true;
    }

    // Define out_min and out_max
    out_min = (t_min * PWM_MAX_DUTY)/period;
    out_max = (t_max * PWM_MAX_DUTY)/period;

    // Set frequency of specified pin
    return SetPinFrequencySafe(pin, 1e6/period);
}

// Write to PWM using the value in custom range
void MotorPWM::writeValue(float value) {
    // Map value to pwm range
    pwm_out = mapFloatPWM(value);

    // Saturate limits in case user screws up
    if(pwm_out > out_max) {
        pwm_out = out_max;
    }
    else if(pwm_out < out_min) {
        pwm_out = out_min;
    }

    // Write to pwm
    pwmWriteHR(pin, pwm_out);
}

// Get PWM output
uint16_t MotorPWM::getPWMOutput() {
    return pwm_out;
}

// Maps (float) from [in_min, in_max] to (int) in [out_min, out_max]
uint16_t MotorPWM::mapFloatPWM(float value) {
    return (float)(value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Debug
void MotorPWM::printSettings() {
    // Print PWM period
    //Serial.print("period: ");
    //Serial.println(period);

    // Print pwm pulse range 
    //Serial.print("t_range: ");
    //Serial.print(t_min); Serial.print(" ");
    //Serial.println(t_max);

    // Print user defined range
    //Serial.print("in_range: ");
    //Serial.print(in_min); Serial.print(" ");
    //Serial.println(in_max);

    // Output range
    //Serial.print("out_range: ");
    //Serial.print(out_min); Serial.print(" ");
    //Serial.println(out_max);      
}