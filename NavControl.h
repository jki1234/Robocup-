#pragma once

/*
 * Navigation control system.
 *
 * This converts the IMU and track motor encoder values into an x,y position.
 */


void navInit(bool isBaseOnRight);

/**
 * Update the navigation system's state.
 *
 * This should be called frequently, as not doing so will lead to increased drift.
 */
void navUpdate();

/**
 * Drive towards the next target weight.
 */
bool navDrive(bool pickNewTargets);

void navGetPosition(float *outX, float *outY);

void navReturnToBase();

/**
 * Call this when you're driving round while collecting a weight.
 *
 * This marks a weight as collected if we drive close to it.
 */
void navExternalDriveMark();
