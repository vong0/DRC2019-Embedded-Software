#ifndef POSE_H
#define POSE_H

#include "Config.h"

// Position of the car
typedef struct pose {
    uint32_t t;     // ms
    float x;        // m
    float y;        // m
    float yaw;      // rad
    float v;        // m/s
} Pose;

// Path to follow
typedef struct pathData {
    uint8_t N;                  // Number of values
    float cx[MAX_PATH_SIZE];    // x array of path
    float cy[MAX_PATH_SIZE];    // y array of path
} PathData;

// Inputs for ODE integration
typedef struct poseInput {
    float v;        // m/s
    float yaw;      // rad/s
    float dt;       // s
} PoseInput;

// Inputs to control motors
typedef struct controlInput {
    float v;        // m/s
    float w;        // rad/s
} ControlInput;

#endif
