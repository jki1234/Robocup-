#pragma once

#include <Arduino.h>
#include <stdint.h>

void wall_avoid(uint16_t top_left, uint16_t top_right, uint16_t top_front);
bool updateWeightFSM();
void updateMasterFSM();


#define KP_L 19
#define KI_L 0.0001
#define KD_L 0.01

#define KP_R 12
#define KI_R 0.01
#define KD_R 0.1


#define ULTRASONIC_THRESHOLD 16
#define TOF_UPPER_THRESHOLD 1
#define TOF_LOWER_THRESHOLD 1

#define TOF_LEFT_THRESHOLD 160
#define TOF_RIGHT_THRESHOLD 160
#define TOF_FRONT_THRESHOLD 100

#define WALL_DISTANCE 190 // Distance from the wall to follow at

#define COUNT_DOWN_TIME 6

#define DEGREES_120 345
#define DEGREES_60 345 / 2

// States
enum { ROAMING, WEIGHT_DETECTED, WEIGHT_COLLECTION, RETURN_TO_BASE };
enum { NONE, LEFT, RIGHT }; // Direction a weight was spotted. Direction is NONE by default.

void startCountdown();

void TOF_response();
void wall_avoid();


void follow_wall_left();

void weight_detection_sequence();

void weight_collection_sequence();


void state_handler();