#ifdef INTEGRATED_SYSTEM

// Libraries
#include "src/RobotBase.h"
#include "src/Stanley.h"
#include "src/Euler.h"
#include "src/PathReader.h"

// Robot
RobotBase robot;

// Odometry
Pose pose;
Euler ode;

// Path tracker
Stanley stanley;

// NUC to STM32 serial writer/loader
PathReader pathReader;

// Serial RX interrupt
static void char_received( uint8_t c ) {
  pathReader.readPathData(c);
}

// EMERGENCY STOP

void setup() {
  // Setup serial
  pathReader.begin(230400);
  nucSerial.attachInterrupt( char_received );

  // Setup robot
  robot.begin();
  robot.setSpeed(0, 0);

  // Serial debug
  #ifdef DEBUG
    debugSerial.println(F("Starting program"));
  #endif

  // Led debug
  pinMode(EMERGENCY_STOP_PIN, INPUT);
}

// State machine for motor controllers
uint8_t stateTrack, statePoseSend;

void loop() {
  static uint32_t sampleLast = millis();
  // Update pose and control speed
  if(robot.sampleData()) {
    // Control speed
    robot.drive();
    
    // Update position
    pose = ode.integrate(robot.getVcm(), robot.getYaw(), robot.getDeltaTime());

    #ifdef DEBUG  // Print data
      logTime();
    #endif
    
    stateTrack++; statePoseSend++;

  }

  // Update path tracker
  if(stateTrack >= 2) {
    // Run stanley controller
    ControlInput speedInput = stanley.calcControlEffort(pose);
    
    // Set speed inputs
    if(!digitalRead(EMERGENCY_STOP_PIN)) {
      robot.setSpeed(speedInput);
    }

    // Reset pose
    stateTrack = 0;
  }

  if(digitalRead(EMERGENCY_STOP_PIN)) {
    robot.setSpeed(0,0);
  }

  // Read path
  if(!pathReader.readStatus()) {
    stanley.setPath(*pathReader.getPathDataPtr());
    pathReader.setReadFlag();

    #ifdef DEBUG
      debugSerial.println("Success\n");
      pathReader.printPathData();
    #endif
  }

  // Send pose via serial
  if(statePoseSend >= 1) {
    pathReader.sendPose(pose);
    statePoseSend = 0;
  }
}

// Prints data
void printData() {
  // Print position (m)
  debugSerial.print(pose.x, 5); debugSerial.print(", ");
  debugSerial.print(pose.y, 5); debugSerial.print(", ");
  debugSerial.print(stanley.getCurrTargetIdx()); debugSerial.print(", ");
  logTime();
  debugSerial.println();  
}

#endif
