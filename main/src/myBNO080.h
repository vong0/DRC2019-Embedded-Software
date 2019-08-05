
#ifndef MYBNO080_H
#define MYBNO080_H

#include "Arduino.h"
#include "SparkFun_BNO080_Arduino_Library.h"
#include "Wire.h"
#include "math.h"

// pitch-roll-yaw (angle_x, angle_y, angle_z)

#define IMU_RESET_PIN 4

class myBNO080: public BNO080 {
private:
    // Data sampled using parent class methods
    float pitch, roll, yaw;         // Euler angles: pitch, roll, yaw
    float q[4];                     // Quaternion angles: r, i, j, k

    // Most recent sampled time
    uint32_t lastUpdateTime;    // To reset IMU if inactive for too long
    uint32_t T;                 // update interval (ms)
    uint32_t resetTime;         // reset time (ms);

    // I2C pins for hardware reset
    uint8_t _rst;
public:
    // Define RESET pins - by default softReset is turned OFF
    myBNO080(uint32_t _T, uint32_t _resetTime = -1);

    // Reset IMU and settings
    void begin();
    void reset();
    void hardReset();

    // Updates derived variables
    bool update();                  // Update angles before getting them
    void updateLastTime();          // Update last update time

    // Get data
    float getRoll();
    float getPitch();
    float getYaw();
    uint32_t getLastTime();

    // Get angles in degrees
    float getRollDeg();
    float getPitchDeg();
    float getYawDeg();

    // Quarternion to Euler conversion
    void qToEuler();
    float normalizeAngle(const float& angle);
};

#endif
