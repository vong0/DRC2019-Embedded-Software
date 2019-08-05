#ifndef ENCODER_H
#define ENCODER_H

#include "Arduino.h"

// Encoder definitions
#define LEFT_ENCODER 0    // pins: 18, 19
#define RIGHT_ENCODER 1   // pins: 2 , 3

// ATMEGA - int 2 bytes (65535 max range)
#define OVERFLOW 65535

class Encoder {
private:
  // Define number
  bool encoderNo;

  // Define states (pinB:pinA, 0 = HIGH, 1 = LOW)
  uint8_t state = 0x00; 
  uint16_t store = 0x00;
  int16_t ticks = 0;

  // Interrupt control
  static const int8_t encStates[16];  // encoder state table (using DIR encoding)
  static Encoder *instances[2];       // stores instances of encoders
  static const uint8_t quadA[2];      // stores pinA of encoder
  static const uint8_t quadB[2];      // stores pinB of encoder

  // Encoder interrupts
  static void rotateLeft();
  static void rotateRight();

  // Helper functions
  int8_t readEncoder();   // reads encoder direction
  void updateTicks();     // updates ticks based on direction

public:
  // Config
  Encoder(bool _encoderNo);   // define encoder number
  void begin();               // init encoder channels

  // Usage: sample encoder ticks
  int16_t sampleTicks();
};

#endif
