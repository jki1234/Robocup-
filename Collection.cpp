#include "Collection.h"
#include "MotorControl.h"
#include "Sensors.h"

int weights_onboard = 0;

void revolver_init() {
  pinMode(DirectionSignal, OUTPUT); // Setup the direction and step as outputs, pin 7 and 8, read from serial ports of teensy
  pinMode(StepSignal, OUTPUT);
}

// If the prox sensor detects a weight(value=1), then collect system starts
bool is_weight_detected() {
  return isIPSOnMetal();
}
/*
void weight_collect() {
  if (is_weight_detected()) {
    Serial.print("Weight collection in progress");
    delayMicroseconds(1000); // Delay to give the weight time to be driven over and fully stored in revolver
    run_revolver(0);
  } else {
    Serial.print("No weight detected yet ...");
  }
}
*/

/**
 * Stepping Function for the revolver 120 degrees
 * 1.8 degrees per step = 67 steps
 **/
void run_revolver(int NUM_STEPS) {
  // Set direction HIGH for clockwise and LOW for counterclosewise
  // Only want to ever rotate one direction, for simplicity and also to
  // reduce chance of errors and slippage, set HIGH=5v
  digitalWrite(DirectionSignal, HIGH);
  // Initialise step to low to ensure no float or error
  digitalWrite(StepSignal, LOW);

  for (int i = 0; i < NUM_STEPS; i++) {
    // Just writing comments for my own remeberance LOL

    // Give one step pulse signal for one step, stepper motors step on rising edge
    digitalWrite(StepSignal, HIGH);
    delayMicroseconds(850);
    // Step low to complete pulse and reset pin
    digitalWrite(StepSignal, LOW);
    delayMicroseconds(850);
  }
}
