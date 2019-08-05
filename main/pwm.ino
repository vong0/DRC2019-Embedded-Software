/*
 * Written by Matthew Vong
 * Unit test for PWM output
 * USE: Takes in an input from Serial and outputs PWM signal
 * Must specify the pin in MotorPWM constructor
 * Must specify the timer of the pin in Motor.begin(timer)
 * All time units are in microseconds
 */

#ifdef PWM

#include "src/MotorPWM.h"

// PWM pins
const int pin = 12;

// PWM configurations
int period = 3000;  // microseconds
int t_min = 1000;   // microseconds
int t_max = 2000;   // microseconds

// User defined range
int in_min = -1024;
int in_max = 1024;

// Construct MotorPWM object
MotorPWM left(pin);

void setup() {
  // Setup Serial
  NeoSerial.begin(115200);
  NeoSerial.println("Starting program");

  // Configure pwm settings
  left.setInputPulse(period, t_min, t_max);
  left.setRange(in_min, in_max);
  left.begin();

  // Print settings
  NeoSerial.println("Config finished");
  left.printSettings();

  // Set PWM to middle of input range
  left.writeValue(0);
}

// Variables to take in Serial inputs
float input = 0;
bool flag = false;
uint16_t pwm_output;

void loop() {
  // Take in Serial input if available
  while(NeoSerial.available() > 0) {
    input = NeoSerial.parseFloat();
    flag = true;
  }

  // Change pwm_output according to input
  if(flag) {
    left.writeValue(input);
    pwm_output = left.getPWMOutput();
    
    NeoSerial.print("Input: "); 
    NeoSerial.print(input); NeoSerial.print(" ");
    NeoSerial.println(pwm_output);
    flag = false;
  }
}

#endif
