#include "Stanley.h"

Stanley::Stanley() {
}

// Sets path
void Stanley::setPath(const PathData &_path) {
    // Copy path
    memcpy(&path, &_path, sizeof(PathData));

    // Determine cyaw (back difference)
    for(uint8_t i = 0; i < path.N; i++) {
        cyaw[i] = atan2(path.cy[i+1] - path.cy[i],
                        path.cx[i+1] - path.cx[i]);
    }

    // Set last index
    cyaw[path.N-1] = cyaw[path.N-2];

    // Reset targets
    currTargetIdx = 0;
}

// Calculate and get control effort
ControlInput Stanley::calcControlEffort(Pose robot) {
    // Stop if close enough
    if(currTargetIdx >= path.N-STOP_INDEX) {
        controlEffort.v = controlEffort.w = 0;
    }
    // Find steering effort otherwise
    else {
        // Add LOOK_AHEAD
        robot.x += LOOK_AHEAD * cos(robot.yaw);
        robot.y += LOOK_AHEAD * sin(robot.yaw);

        // Get steering angle
        float delta = calcSteeringAngle(robot);

        // Get angular speed
        controlEffort.w = calcAngularSpeed(pidAngle(delta));

        // Regulate speed
        controlEffort.v = regulateSpeed(controlEffort.w);
    }

    return controlEffort;
}

// Gets control effort
float Stanley::calcSteeringAngle(const Pose& robot) {
    // Find the target and update e_fa
    float e_fa = calcTargetError(robot);

    // theta_e corrects the heading error
    float theta_e = normalizeAngle(cyaw[currTargetIdx] - robot.yaw);

    // theta_d corrects the cross track error
    float theta_d = atan2(STANLEY_GAIN * e_fa, robot.v + Ksoft);

    // Prevent theta_d from saturating
    if (theta_d < -MAX_THETA_D) {
        theta_d = -MAX_THETA_D;
    } else if (theta_d > MAX_THETA_D) {
        theta_d = MAX_THETA_D;
    }

    // Steering control (delta)
    return theta_e + theta_d;
}

// PID controller to get angular velocity
float Stanley::pidAngle(const float& _delta) {
    static float lastDelta = 0;
    static float integral = 0;
    static unsigned long prevTime = micros();

    unsigned long currTime = micros();
    float dt = (float) (currTime - prevTime)/1000000.0;

    integral += _delta * dt;
    //TODO limit integral

    float diff = (_delta - lastDelta)/dt;

    float p = Kw_P * _delta;
    float i = Kw_I * integral;
    float d = Kw_D * diff;

    lastDelta = _delta;
    prevTime = currTime;

    return p + i + d;
}

// Regulate speed based on angular speed
float Stanley::regulateSpeed(const float& _w) {
    // Decrease speed on large rotations
    // return V_MIN + (1 - abs(_w/W_MAX)) * (V_MAX - V_MIN);
    // return V_MIN + (1- exp( abs(_w/W_MAX) )) * (V_MAX - V_MIN);
    // return V_MIN + (1- exp( -abs(_w/W_MAX) ) * REDUCTION_FACTOR) * (V_MAX - V_MIN);
    
    float v;
    float w = abs(_w);
    // Dead zone LOW
    if(w <= W_DEAD_LOW) {
        v = V_MAX;
    }
    // Dead zone high
    else if(w >= W_DEAD_HIGH) {
        v = V_MIN;
    }
    // Middle value
    else {
        float m = (V_MAX - V_MIN) / (W_DEAD_HIGH - W_DEAD_LOW);
        v = V_MAX - m * (w - W_DEAD_LOW);
    }
    return v;
    
}

// Calculate angular speed from steering angle
float Stanley::calcAngularSpeed(const float& _delta) {
    float w = _delta;

    // Saturate w if out of bounds
    if(w < -W_MAX) {
        w = -W_MAX;
    }
    else if(w > W_MAX) {
        w = W_MAX;
    }

    return w;
}

// Gets target index and e_fa
float Stanley::calcTargetError(const Pose& robot) {
    // Project RMS error onto the front axle vector
    float dx = path.cx[currTargetIdx] - robot.x;
    float dy = path.cy[currTargetIdx] - robot.y;

    // Update if next points are closer
    for(int i = 1; i <= SEARCH_CLOSEST; i++) {
        findClosestPoint(dx, dy, robot, i);
    }

    // Calculate normal distance e_fa
    return dx * (-sin(robot.yaw)) + dy * (cos(robot.yaw));
}

// Search for closer point and updates dx and dy
void Stanley::findClosestPoint(float &dx, float &dy, const Pose& robot, uint8_t idx) {
    // Update if next point is closer
    if(currTargetIdx < path.N - idx) {
        float dx1 = path.cx[currTargetIdx+idx] - robot.x;
        float dy1 = path.cy[currTargetIdx+idx] - robot.y;

        if(dx1*dx1 + dy1*dy1 <= dx*dx + dy*dy) {
            currTargetIdx++;
            dx = dx1; dy = dy1;
        }
    }
}

// Normalise angle to [-pi, pi]
float Stanley::normalizeAngle(const float& angle) {
    float wrappedAngle = angle;
    while(wrappedAngle > PI) {
        wrappedAngle -= 2*PI;
    }
    while(wrappedAngle < -PI) {
        wrappedAngle += 2*PI;
    }
    return wrappedAngle;
}

// Debug
uint8_t Stanley::getCurrTargetIdx() {
    return currTargetIdx;
}
