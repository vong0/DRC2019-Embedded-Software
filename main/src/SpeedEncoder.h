#ifndef SPEED_ENCODER_H
#define SPEED_ENCODER_H

#include "Arduino.h"
#include "Encoder.h"
#include "math.h"

// SpeedEncoder class to measure the speed of an encoder
//   - Ticks are SAMPLED using the Encoder class

class SpeedEncoder: public Encoder {
private:
    // Tick samples to get speed
    int16_t prevTicks;
    int16_t currTicks;

    // Tick sample times
    uint32_t prevTime;     // microseconds
    uint32_t currTime;     // microseconds
    uint32_t T;            // microseconds

    // Speed
    float tickSpeed;       // counts/s
    uint16_t tickRatio;    // counts/rev

public:
    // Config
    SpeedEncoder(bool _encoderNum, uint16_t _tickRatio, uint32_t _T);

    // USAGE: sample encoder data
    bool sampleData();              // samples data - updates variables
    
    // Getters speed data
    float getTickSpeed();           // counts/s
    float getRadianSpeed();         // rad/s
    uint32_t getDeltaTime();        // microseconds
    uint32_t getCurrTime();         // microseconds

    // Debug variables
    int16_t getCurrTicks();            // count/s
    int16_t getPrevTicks();            // counts/s
    int16_t getDeltaTicks();           // count/s

    // Helper functions
    void calculateSpeed();
    float ticksToRad(float w_ticks);
    float radToTicks(float w_rad);
};

#endif