#ifdef ROBOT_DRIVER

#include "src/RobotBase.h"

// Create robot
RobotBase robot;

void setup() {
  // Setup Serial
  NeoSerial.begin(115200);
  NeoSerial.println("Starting program");

  // Setup robot
  robot.begin();
  robot.setSpeed(0, 0);
}

void loop() {
  // Update every SAMPLE_TIME
  if(robot.sampleData()) {
    robot.drive();
    printData();

    // Log time
    //logTime();
  }

  // Load data from serial
  readSerial();
}

// Load data
void readSerial() {
  // Variables
  float v, w;
  bool flag = false;
  
  // Take in Serial input if available
  while(NeoSerial.available() > 0) {
    v = NeoSerial.parseFloat();
    w = NeoSerial.parseFloat();
    flag = true;
  }

  // Change pwm_output according to input
  if(flag) {
    robot.setSpeed(v, w);
    flag = false;
  }  
}

// Prints data
void printData() {
  // Get data
  float v_d = robot.getVd();
  float v_cm = robot.getVcm();
  float w_d = robot.getWd();
  float w_cm = robot.getWcm();
  float yaw = robot.getYawDeg();

  // Print data
  NeoSerial.print(v_d); NeoSerial.print(", ");
  NeoSerial.print(v_cm); NeoSerial.print(", ");
  NeoSerial.print(w_d); NeoSerial.print(", ");
  NeoSerial.print(w_cm); NeoSerial.print(", ");
  NeoSerial.print(yaw);
  NeoSerial.println();  
}
#endif
