#ifdef ENCODER
#include "src/Encoder.h"

// For Arduino MEGA - pins 2,3,18,19, (20,21 - used for i2c)
Encoder leftEncoder(LEFT_ENCODER);  // pins 2,3
Encoder rightEncoder(RIGHT_ENCODER); // pins 18,19

void setup() {
  // Set up serial
  NeoSerial.begin(115200);
  
  // Set up encoder
  leftEncoder.begin();
  rightEncoder.begin();
}

void loop() {
  // Print counts of each encoder
  NeoSerial.print("ticks: "); 
  NeoSerial.print(leftEncoder.sampleTicks());
  NeoSerial.print(", ");
  NeoSerial.print(rightEncoder.sampleTicks());
  NeoSerial.println();
}

#endif
