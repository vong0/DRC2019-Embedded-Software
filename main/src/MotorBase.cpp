#include "MotorBase.h"

MotorBase::MotorBase(bool x, uint32_t T)
    : enc(x, TICK_RATIO, T), pwm(pwmPin[x]) {
}

// Initialise motors
void MotorBase::begin(){
    // Initialise encoders
    enc.begin();

    // Initialise pwm
    pwm.setInputPulse(PWM_PERIOD, PWM_PULSE_LOW, PWM_PULSE_HIGH);
    pwm.setRange(CONTROL_MIN, CONTROL_MAX);
    pwm.begin();

    // Set controller limits
    cont.setRange(CONTROL_MIN, CONTROL_MAX);
}

// Sets controller gains
void MotorBase::setGains(float *e_in, uint8_t _N_e, float *m_in, uint8_t _N_m) {
    cont.setGains(e_in, _N_e, m_in, _N_m);
}

// Sets encoder speed in rad/s
void MotorBase::setRadianSpeed(float w) {
    w_d = w;
}

// Controls pwm to get desired speed rad/s
void MotorBase::controlSpeed() {
    // Write to PWM
    pwm.writeValue(m);
}

// Samples data - updates all variables
// Returns true if data was updated
bool MotorBase::sampleData() {
    if(enc.sampleData()) {
         w_a = enc.getRadianSpeed();     // get actual w_a (rad/s)
         e = w_d - w_a;                  // get error in w (rad/s)
         m = cont.getControlEffort(e);   // get control effort
        return true;
    }
    return false;
}

// Get speed in counts/s
float MotorBase::getTickSpeed(){
    return enc.getTickSpeed();
}

// Get current time in microseconds
uint32_t MotorBase::getCurrTime(){
    return enc.getCurrTime();
}

// Get difference in sample times in seconds
float MotorBase::getDeltaTime(){
    return (float)enc.getDeltaTime()/1e6;
}

// Gets desired speed in rad/s
float MotorBase::getDesiredRadianSpeed() {
    return w_d;
}

// Get speed in rad/s
float MotorBase::getRadianSpeed(){
    return w_a;
}

// Getter speed error in RAD/S
float MotorBase::getRadianError() {
    return e;
}

// Gets control effort - updated from sampleData()
float MotorBase::getControlEffort() {
    return m;
}

// Print motor configs
void MotorBase::printSettings(){
    pwm.printSettings();
    cont.printSettings();
}

// Write to PWM using the control range
void MotorBase::writeValue(float value){
    pwm.writeValue(value);
}

// Gets PWM output
uint16_t MotorBase::getPWMOutput() {
    return pwm.getPWMOutput();
}

// Helper function - gets control effort for given error
float MotorBase::getControlEffort(float e) {
    return cont.getControlEffort(e);
}

// Gets the current encoder ticks
int32_t MotorBase::getCurrTicks() {
    return enc.getCurrTicks();
}

// Gets the previous encoder ticks
int32_t MotorBase::getPrevTicks() {
    return enc.getPrevTicks();
}

// Get difference in counts between samples
int32_t MotorBase::getDeltaTicks(){
    return enc.getDeltaTicks();
}
