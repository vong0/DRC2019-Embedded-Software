#ifndef CONFIG_H
#define CONFIG_H

#include "Arduino.h"
#include <NeoHWSerial.h>

#define EMERGENCY_STOP_PIN 13

// ----Motor----
// Motor configurations (LEFT_MOTOR, RIGHT_MOTOR)
static const uint8_t pwmPin[2] = {6, 7};

// Motor control parameters
static const uint8_t motor_N_e = 3, motor_N_m = 2;
static const float motor_e_gains[] = {64.6221109336773, -107.0597795006965, 44.2599059086172};
static const float motor_m_gains[] = {1.688682860224775, -0.688682860224775};

// Wheel parameters
#define WHEEL_RADIUS 0.05       // m
#define MAX_RADIAN_SPEED 34.00  // rad/s
#define MAX_WHEEL_SPEED WHEEL_RADIUS * MAX_RADIAN_SPEED

// ----Controller---- = MUST have size >= max(N_e, N_m)
#define CONTROL_MAX_HISTORY 3   // history of e and m to store
#define RING_BUFFER_SIZE 3      // ring buffer used in controller

// ----Robot----
// Car parameters
#define WHEEL_DIST 0.1825       // distance between wheel centres (m)

// Sample times
#define SAMPLE_TIME 8e3         // microseconds (motors + IMU)
#define RESET_TIME -1           // microseconds (IMU) --> (-1 = inf)

// ----Path----
#define MAX_PATH_SIZE 40        // max points stored

// ----Serial ports----
#define debugSerial NeoSerial3
#define nucSerial NeoSerial
#define BAUD_RATE 230400

// ----Path tracking----
#define STANLEY_GAIN 1.0    // higher corrects track error but oscillates
#define REDUCTION_FACTOR 2
#define TEMP_MULT 3.0
#define Kw_P 4.0 * TEMP_MULT
#define Kw_I 0.5 * TEMP_MULT
#define Kw_D 0.5
#define Ksoft 0.01          // m/s
#define V_MIN 0.8           // m/s
#define V_MAX 1.6           // m/s
#define W_MAX 10           // rad/s
#define W_DEAD_LOW 1        // rad/s
#define W_DEAD_HIGH 6       // rad/s
#define LOOK_AHEAD 0.40        // m
#define SEARCH_CLOSEST 5    // searches up to the next x points to find the closest one
#define STOP_INDEX 2        // stop at path index from end

#define MAX_THETA_D 25 *3.14159/180      // Max heading error contributed by distance from path

#endif
