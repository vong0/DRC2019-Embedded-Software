#ifdef NUC_SERIAL
#include "src/PathReader.h"

// NUC to STM32 serial writer/loader
PathReader pathReader;

// Fill up pose of robot
Pose robot;

static void char_received( uint8_t c ) {
  pathReader.readPathData(c);
}

void setup() {
  // Start up serial ports
  pathReader.begin(115200);
  nucSerial.attachInterrupt( char_received );
  debugSerial.println(F("Starting Receive Program"));
}

void loop() {
  // Read path
  if(!pathReader.readStatus()) {
    debugSerial.println("Success\n");
    pathReader.setReadFlag();
    pathReader.printPathData();
  }
}

#endif
