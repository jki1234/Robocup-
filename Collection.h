#pragma once 

#include <Arduino.h>
#include <Servo.h>
#include <stdint.h>


/** 
* Pins for stepper motor defined, serial ports on cpu
**/
#define DirectionSignal 7
#define StepSignal 8



extern int weights_onboard;


/** 
 * Intialise the revolver direction and step signals 
 *
 */
void revolver_init();
/**
 * Checks if the weight is detected
 */
bool is_weight_detected();
void weight_collect();
/**
 * Function for powering and running the stepper motor
 * 
 * Runs the revolver system, pass argument [num_steps]
 */
void run_revolver(int NUM_STEPS);
