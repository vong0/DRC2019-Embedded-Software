
// Unit tests
//#define IMU
//#define ENCODER
//#define SPEED_ENCODER
//#define PWM
//#define CONTROLLER

// Single motor integration tests
//#define MOTOR_IDENTIFY  // motors with no controller
//#define MOTOR_INTEGRATED  // motors with controller
//#define MOTOR_LINEAR_SPEED  // motors in m/s

// Combined motor integration tests
//#define ROBOT_DRIVER  // drives robot using v and w

// Odometry tests
//#define ODOMETRY // finds the position of the robot

// Path tracking tests
//#define STANLEY_SIMULATION
//#define STANLEY

// Serial tests
//#define NUC_SERIAL

// Integrated code
#define INTEGRATED_SYSTEM

// Debug statements
//#define DEBUG

// ----Benchmark function----
// Log time between next occurence
#include <NeoHWSerial.h>
void logTime() {
  static uint32_t prevTime, currTime;
  prevTime = currTime;
  currTime = micros();
  NeoSerial.println(currTime-prevTime);
}
