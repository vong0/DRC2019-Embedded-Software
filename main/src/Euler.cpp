#include "Euler.h"

// Sets state
void Euler::setState(Pose state) {
    memcpy(&pose, &state, sizeof(Pose));
}

// Integrates ODE
Pose Euler::integrate(float v, float angle, float dt) {
    // Integrate [x, y, angle]
    float ds = dt*v;
    pose.x += ds*cos(angle);     // x
    pose.y += ds*sin(angle);     // y
    pose.yaw = angle;            // angle
    pose.v = v;
    pose.t = millis();
    return pose;
}



