#include "SpeedEncoder.h"

// Define timer number, tick ratio, sample time
SpeedEncoder::SpeedEncoder(bool _encoderNum, uint16_t _tickRatio, uint32_t _T)
    : Encoder(_encoderNum), tickRatio(_tickRatio), T(_T) {
}

// Sample the encoder speed at T
// Returns true if data is sampled/updated
bool SpeedEncoder::sampleData() {
    if(micros() - currTime >= T) {
         // Update ticks
         prevTicks = currTicks;
         currTicks = sampleTicks();
    
         // Update time
         prevTime = currTime;
         currTime = micros();
    
         // Calculate speed
         calculateSpeed();
    
        return true;
    }
    return false;
}

// Get encoder speed in COUNTS/S
float SpeedEncoder::getTickSpeed() {
    return tickSpeed;
}

// Get encoder speed in RAD/S
float SpeedEncoder::getRadianSpeed() {
    return ticksToRad(tickSpeed);
}

// Gets latest sample time
uint32_t SpeedEncoder::getCurrTime() {
    return currTime;
}

// Gets difference in sample times
uint32_t SpeedEncoder::getDeltaTime() {
    return currTime-prevTime;
}

// Get current ticks
int16_t SpeedEncoder::getCurrTicks() {
    return currTicks;
}

// Get prev ticks
int16_t SpeedEncoder::getPrevTicks() {
    return prevTicks;
}

// Gets difference in ticks
int16_t SpeedEncoder::getDeltaTicks() {
    return currTicks-prevTicks;
}

// Calculates speed using current sample data
void SpeedEncoder::calculateSpeed() {
    // Calculate dTicks first
    tickSpeed = (float) currTicks - prevTicks;

    // CASE 1: underflow --> add overflow amount
    if(tickSpeed <= -OVERFLOW/2) {
        tickSpeed += OVERFLOW + 1;
    }
    // CASE 2: overflow --> subtract overflow difference
    else if(tickSpeed >= OVERFLOW/2) {
        tickSpeed -= OVERFLOW + 1;
    }

    // Calculate speed
    tickSpeed = 1e6*tickSpeed/(currTime-prevTime);
}

// Conversion functions
float SpeedEncoder::ticksToRad(float w_ticks) {
    return (float)tickSpeed * 2*PI/tickRatio;
}

float SpeedEncoder::radToTicks(float w_rad) {
    return (float)tickSpeed * tickRatio/(2*PI);
}
