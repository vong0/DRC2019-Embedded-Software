/*
 * This is a speed unit test for the STM32F103
 * Specify the tickRatio, SAMPLE_TIME and QUAD_TIMER_x (timers with encoder mode)
 * See the "SpeedEncoder.h" file for the pinouts for each quadrature encoder timer
*/
#ifdef SPEED_ENCODER

#include "src/SpeedEncoder.h"

#define tickRatio 1920
#define SAMPLE_TIME 8e3   // microseconds

SpeedEncoder leftEncoder(LEFT_ENCODER, tickRatio, SAMPLE_TIME);
SpeedEncoder rightEncoder(RIGHT_ENCODER, tickRatio, SAMPLE_TIME);

// Function def
void printData(SpeedEncoder enc);

void setup() {
  // Start up serial
  NeoSerial.begin(115200);

  // Config encoders
  leftEncoder.begin();
  rightEncoder.begin();
}

void loop() {
  // Update every SAMPLE_TIME
  if(leftEncoder.sampleData()) {
    printData(leftEncoder);
  }
  if(rightEncoder.sampleData()) {
    printData(rightEncoder);
  }
}

// Prints data
void printData(SpeedEncoder enc) {
  // Get data
  float w = enc.getTickSpeed();
  float t = enc.getCurrTime();

  // Print details
  NeoSerial.print(t); NeoSerial.print(", ");
  NeoSerial.print(w); NeoSerial.print(", ");
//  NeoSerial.print(enc.getPrevTicks()); NeoSerial.print(", ");
//  NeoSerial.print(enc.getCurrTicks()); NeoSerial.print(", ");
//  NeoSerial.print(enc.getDeltaTicks()); NeoSerial.print(", ");
//  NeoSerial.print(enc.getDeltaTime()); NeoSerial.print(", ");
  NeoSerial.println();
}

#endif
