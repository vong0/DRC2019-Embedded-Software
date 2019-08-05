#ifndef STANLEY_H
#define STANLEY_H

#include "Arduino.h"
#include "math.h"
#include "Pose.h"

class Stanley {
private:
    // Steering outputs
    ControlInput controlEffort; // {m/s, rad/s}

    // Path to follow
    PathData path;

    // Derived values
    float cyaw[MAX_PATH_SIZE];  // tangent of path (from cx, cy)
    uint8_t currTargetIdx;      // index of curr target

public:
    Stanley();

    // Sets path to follow
    void setPath(const PathData &_path);

    // Calculates and returns control effort
    ControlInput calcControlEffort(Pose robot);

    // Helper functions (controlEffort)
    float calcSteeringAngle(const Pose& robot);
    float regulateSpeed(const float& _w);
    float calcAngularSpeed(const float& _delta);

    // Helper functions (steering angle)
    float calcTargetError(const Pose& robot);       // finds target e_fa
    void findClosestPoint(float &dx, float &dy, const Pose& robot, uint8_t idx);    // looks at idx index and updates if closer
    float normalizeAngle(const float& angle);       // normalise angle to [-pi, pi]
    float pidAngle(const float& _delta);

    // Debug
    uint8_t getCurrTargetIdx();
};

#endif
