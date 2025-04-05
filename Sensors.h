#pragma once

#include <stdint.h>

#define START_BUTTON 20 

/**
 * Stores all the ToF sensor readings, in units of millimetres.
 */
struct ToFSensorData {
  // This must match up with the shortRangePins/longRangePins defined in Sensors.cpp

  // Short-range ToF sensors
  uint16_t TOP_FRONT;

  // Long-range ToF sensors
  uint16_t UPPER_LEFT;
  uint16_t UPPER_RIGHT;
  uint16_t TOP_LEFT;
  uint16_t TOP_RIGHT;
  uint16_t LOWER_LEFT; // Mounted on the weight scraper
  uint16_t LOWER_RIGHT;
};

void SetupSensors();
void UpdateSensors();
void DebugSearchSensors();

/**
 * Get the time-of-flight sensor readings.
 *
 * This never causes I2C traffic.
 */
ToFSensorData *getTOFSensorValues();

/**
 * Get the yaw of the robot in radians.
 *
 * This ranges from -pi to pi. Positive values are counter-clockwise, when the robot
 * is viewed from above.
 */
float getIMUYaw();

/**
 * Get the robot's pitch in radians, as measured by the IMU.
 *
 * Zero means the robot is level, positive means the front has been lifted (eg if it's
 * driving up a ramp), negative values means the robot's rear has been lifted.
 */
float getIMUPitch();

/**
 * Get the robot's bank angle in radians, as measured by the IMU.
 *
 * Zero means the robot is level, positive means it's banking to the left, negative
 * means it's banking to the right.
 */
float getIMUBank();

/**
 * Find the difference between two angles (a-b) while accounting for wrap-around at the -pi/pi degree point.
 */
float angleDifference(float a, float b);

/**
 * True if the IPS sensor can see a metal weight.
 */
bool isIPSOnMetal();

bool isStartPressed();
