
#include "Encoder.h"

/*
  The following is the typical sequence of code on the output when
  moving from one step to the next:

  Position   Bit1   Bit0
  ----------------------
  Step1       0      0
  1/4         0      1
  1/2         1      1
  3/4         1      0

  Produces a 4x4 = 16 possible states
  We define valid states and invalid states

  For more details visit:
  https://www.best-microcontroller-projects.com/rotary-encoder.html
  https://github.com/buxtronix/arduino/tree/master/libraries/Rotary
*/

// Define which encoder to use
Encoder::Encoder(bool _encoderNo)
  : encoderNo(_encoderNo) {
    instances[encoderNo] = this;
}

// Initialises encoder channels
void Encoder::begin() {
  // Set pins to input and enable pullup resistors
  pinMode(quadA[encoderNo], INPUT_PULLUP);
  pinMode(quadB[encoderNo], INPUT_PULLUP);

  // Attach interrupts
  void (*myISR)();
  switch(encoderNo) {
    case LEFT_ENCODER: myISR = rotateLeft; break;
    case RIGHT_ENCODER: myISR = rotateRight; break;
  }
  attachInterrupt(digitalPinToInterrupt(quadA[encoderNo]), myISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(quadB[encoderNo]), myISR, CHANGE);
}

// Samples encoder ticks
int16_t Encoder::sampleTicks() {
  return ticks;
}

// Encoder interrupt variables
// encStates: 0 = No steps complete, -1 = CW step, 1 = CCW step
Encoder* Encoder::instances[2] = {NULL, NULL};
const uint8_t Encoder::quadA[2] = {2, 18};
const uint8_t Encoder::quadB[2] = {3, 19};
const int8_t Encoder::encStates[16] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};

// Left encoder tick interrupt
void Encoder::rotateLeft() {
  if(Encoder::instances[LEFT_ENCODER] != NULL) {
    Encoder::instances[LEFT_ENCODER]->updateTicks();
  }
}

// Right encoder tick interrupt
void Encoder::rotateRight() {
 if(Encoder::instances[RIGHT_ENCODER] != NULL) {
    Encoder::instances[RIGHT_ENCODER]->updateTicks();
  }
}

// Updates encoder ticks
void Encoder::updateTicks() {
  ticks += readEncoder();
}

// Read encoder direction. Debounce using state history
int8_t Encoder::readEncoder() {
  // Remember previous state (shift it)
  state <<= 2;

  // Add current state
  state |= ((digitalRead(quadA[encoderNo]) << 1) | digitalRead(quadB[encoderNo]));

  // Check valid output = last 2 states
  state &= 0x0f;

  // Single processing = check valid output only
  // Return direction based on current state
  return encStates[state];
}
