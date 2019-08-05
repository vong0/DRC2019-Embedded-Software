/*
 * This is an integration test for the IMU
 * Use SCL = PB6, SDA = PB7
 */

#ifdef IMU

#include "src/myBNO080.h"

// Timing macros
#define IMU_SAMPLE_TIME 8   // milliseconds

// Create IMU object
myBNO080 myIMU(IMU_SAMPLE_TIME); // by default reset time is off

void setup() {
  // Start up serial
  NeoSerial.begin(115200);

  // Configure IMU for sample time
  NeoSerial.println("Starting IMU");
  myIMU.begin();

  // Print out settings
  NeoSerial.println("Rotation vector, accelerometer, enabled");
  NeoSerial.println("Output in form i, j, k, real, accuracy");  
}

void loop() {
  // Sample IMU data if available
  if(myIMU.update()) {
    printData();

    // Log time
//    logTime();
  }
}

// Prints data
void printData() {
  NeoSerial.print(myIMU.getLastTime());
  NeoSerial.print(", ");
  NeoSerial.print(myIMU.getPitchDeg());
  NeoSerial.print(", ");
  NeoSerial.print(myIMU.getRollDeg());
  NeoSerial.print(", ");
  NeoSerial.print(myIMU.getYawDeg());
  NeoSerial.println();  
}

#endif
