#include "Control.h"
#include "Collection.h"
#include "MapParser.h"
#include "MotorControl.h"
#include "NavControl.h"
#include "SDCard.h"
#include "Sensors.h"
#include "Utils.h"

enum wfsm_t {
  WFSM_IDLE, // We haven't seen a weight yet
  WFSM_SEEN, // We know there's a weight
  WFSM_PROX, // We know it's metal
  WFSM_WIGGLE1,
  WFSM_WIGGLE2,
  WFSM_WIGGLE3,
};

// Master FSM
enum mfsm_t {
  MFSM_WAITING_TO_START,
  MFSM_ROAMING,
  MFSM_RETURNING,
  MFSM_DRIVE_TO_CORNER,
  MFSM_DROPPING,
};

static wfsm_t wfsmState = WFSM_IDLE;
static int ipsStartMM = 0;
static unsigned long resumeTime = 0;
static int seenDist = 0;
static bool isRevolverOffset = false; // true means flat side down

// Wall following
static int corner_distance = 0;
static bool wall_following_tt = 1; // Toggles tank turns for wall following code. Should be 1 when just following a wall but 0 for when driving to a corner.
static int left_motor_effort = 0;
static int right_motor_effort = 0;

// Master FSM
static mfsm_t masterState = MFSM_WAITING_TO_START;
static bool hasSampledSide = false;
static bool isHomeOnRight;


// Maximum distance we can detect a weight at
static constexpr int MAX_WEIGHT_DISTANCE_MM = 450;

// The difference in percent between the top and bottom
// sensors that indicate a weight.
static constexpr int MIN_WEIGHT_DISTANCE_DIFF_PERCENT = 30;

// The minimum difference between the sensors to detect
// a weight. This AND the minimum percent must both be met.
static constexpr int MIN_WEIGHT_DISTANCE_DIFF_MM = 300;

// The angle of the front bottom ToF sensor.
// The top one isn't currently fixed in place well.
// This number is from solidworks, where 0 is straight forwards.
static constexpr float SENSOR_ANGLE = 48.31 * 0.85;

// PID variables
float previous_error = 0;
long integral_left = 0;
long integral_right = 0;
unsigned long lastWallUpdateMicros = 0;
//

int clamp(int value, int lower_bound, int upper_bound) {
  if (value < lower_bound)
    return lower_bound;
  if (value > upper_bound)
    return upper_bound;
  return value;
}

int motor_deadzone(int value) {
  if ((value < 0) && (value > -1300)) {
    return -1300;
  }

  if ((value > 0) && (value < 1300)) {
    return 1300;
  }
  return value;
}

void wall_avoid(uint16_t top_left, uint16_t top_right, uint16_t top_front) {
  if (micros() < resumeTime || motorIsTurningOnSpot()) {
    return;
  }

  if (top_left < TOF_LEFT_THRESHOLD) {
    motorSelectSpeeds(2000 + 36 * (TOF_LEFT_THRESHOLD - top_left), 1800);
  }
  if (top_right < TOF_RIGHT_THRESHOLD) {
    motorSelectSpeeds(1800, 2000 + 36 * (TOF_RIGHT_THRESHOLD - top_right));
  }
  if (top_front < TOF_FRONT_THRESHOLD) {
    motorSelectHeading(getIMUYaw() + radians(180), 2400);
    resumeTime = micros() + 2'000'000;
  } else {
    motorSelectHeading(getIMUYaw(), 1900);
  }
}


void follow_wall_left(uint16_t left_tof_reading, uint16_t upper_right_tof_reading) {
  // Calculate the error
  int error = left_tof_reading - WALL_DISTANCE;


  // if (upper_right_tof_reading < 320) {
  //   Serial.println("Large correction triggered due to upper right ToF sensor.");
  // }
  if (wall_following_tt && (upper_right_tof_reading < 320)) {
    // Serial.println("Large correction triggered due to upper right ToF sensor.");
    error += 18 * (upper_right_tof_reading - 285);
  }

  if (error < 0) {
    error = (int)(error * 1.1);
  }


  unsigned long now = micros();
  long elapsed = (long)(now - lastWallUpdateMicros);
  lastWallUpdateMicros = now;

  // Handle micros() overflow
  if (elapsed < 0 || elapsed > 100'000) {
    return;
  }

  // Calculate the integral and derivative
  integral_left += error * elapsed;
  integral_left = clamp(integral_left, -2500, 2500);
  int derivative = (error - previous_error) / elapsed;

  // PID output
  int output = KP_L * error + KI_L * integral_left + KD_L * derivative;


  int left_output = 5300 - output;
  int right_output = 5300 + output;

  if (error < -130) {
    right_output = motor_deadzone(output);
  }

  if (error > 900) {
    left_output = motor_deadzone(-output);
  }

  left_output = clamp(left_output, -5800, 6500);
  right_output = clamp(right_output, -5800, 6500);


  // Update the previous error
  previous_error = error;
  // Update global motor efforts
  left_motor_effort = left_output;
  right_motor_effort = right_output;
  motorSelectSpeeds(left_output, right_output);
  // Serial.print("Motor efforts: ");
  // Serial.print(left_output);
  // Serial.print(" ");
  // Serial.print(right_output);
  // Serial.print("  Current integral error (Left): ");
  // Serial.print(" Current error: ");
  // Serial.print(error);
  // Serial.print(" Current PID output: ");
  // Serial.println(output);
}

void follow_wall_right(uint16_t right_tof_reading, uint16_t upper_left_tof_reading) {
  // Calculate the error
  int error = 1.0 * (right_tof_reading - WALL_DISTANCE);

  if (wall_following_tt && (upper_left_tof_reading < 320)) {
    // Serial.println("Large correction triggered due to upper left ToF sensor.");
    error += 18 * (upper_left_tof_reading - 285);
  }

  if (error < 0) {
    error = (int)(error * 1.0);
  }


  unsigned long now = micros();
  long elapsed = (long)(now - lastWallUpdateMicros);
  lastWallUpdateMicros = now;

  // Handle micros() overflow
  if (elapsed < 0 || elapsed > 100'000) {
    return;
  }

  // Calculate the integral and derivative
  integral_right += error * elapsed;
  integral_right = clamp(integral_right, -2500, 2500);
  int derivative = (error - previous_error) / elapsed;

  // PID output
  int output = KP_R * error + KI_R * integral_right + KD_R * derivative;
  output = clamp(output, -2000, 2000);


  // int left_output = 3500 + output;
  // int right_output = 3500 - output;
  int left_output = 4500 + output;
  int right_output = 4500 - output;

  if (error < -130) {
    left_output = motor_deadzone(output);
  }

  if (error > 700) {
    right_output = motor_deadzone(-output);
  }

  left_output = clamp(left_output, -5500, 5500);
  right_output = clamp(right_output, -5500, 5500);


  // Update the previous error
  previous_error = error;
  // Update global motor efforts
  left_motor_effort = left_output;
  right_motor_effort = right_output;
  motorSelectSpeeds(left_output, right_output);
  // Serial.print("Motor efforts: ");
  // Serial.print(left_output);
  // Serial.print(" ");
  // Serial.print(right_output);
  // Serial.print("  Current integral error (Right): ");
  // Serial.print(" Current error: ");
  // Serial.print(error);
  // Serial.print(" Current TOF : ");
  // Serial.println(right_tof_reading);
}


bool drive_to_corner(uint16_t top_front, uint16_t left_tof_reading, uint16_t upper_right_tof_reading, uint16_t right_tof_reading, uint16_t upper_left_tof_reading) {
  wall_following_tt = 0; // Tank turns are turned off now.
  // Checks that the front of the robot is not already driven into the wall.
  serialPrintf(
      "Top front: %2d Upper right: %2d Upper left: %2d   Left motor = %2d   Right motor = %2d. ", top_front, upper_right_tof_reading, upper_left_tof_reading, left_motor_effort *= 0.5,
      right_motor_effort *= 0.5
  );

  if (top_front < 220) {
    motorSelectSpeeds(left_motor_effort *= 0.95, right_motor_effort *= 0.95);
    bool tofClose = ((left_tof_reading < WALL_DISTANCE + 80) || (right_tof_reading < WALL_DISTANCE + 80)) && top_front < 150;
    bool motorsOff = abs(left_motor_effort) < 200 && abs(right_motor_effort) < 200;
    if (tofClose || motorsOff) {
      Serial.println("In the corner now..");
      motorSelectSpeeds(0, 0);
      corner_distance = motorGetTotalMovementMM();
      wall_following_tt = 1; // Tank turns are turned back on now.
      return false;
    }
  } else {
    // Checks which side of the robot is closest to a wall, and then follows that wall on that side.
    if ((left_tof_reading < (WALL_DISTANCE + 200)) && (upper_right_tof_reading > 150)) {
      Serial.println("Following the wall on LHS");
      follow_wall_left(left_tof_reading, upper_right_tof_reading);
    } else if ((right_tof_reading < (WALL_DISTANCE + 200)) && (upper_right_tof_reading > 150)) {
      Serial.println("Following the wall on RHS");
      follow_wall_right(right_tof_reading, upper_left_tof_reading);
    } else {
      // Go straight if not near a wall
      motorSelectSpeeds(4500, 4500);
    }
    return true;
  }

  // Out in flat ground, no wall in sight?
  return true;
}


bool weight_dropoff() {
  if (weights_onboard == 0) {
    return false;
  }

  // When the robot begins the dropoff with 3 weights,
  // rotate it back from the 60 degree storage
  if (isRevolverOffset) {
    motorImmediateStop();
    delay(500);
    run_revolver(DEGREES_60);
    isRevolverOffset = false;
  }

  // While robot has not reversed 120mm driveback at the heading its parked
  if ((motorGetTotalMovementMM() - corner_distance) < 160) {
    serialPrintf("Distance: %d", motorGetTotalMovementMM() - corner_distance);
    motorSelectHeading(getIMUYaw(), -3000);
    return true;
  }
  serialPrintf("Stopping motors");

  motorImmediateStop();
  delay(500);
  run_revolver(DEGREES_120);
  delay(500);
  weights_onboard--;

  corner_distance = motorGetTotalMovementMM();
  return true;
}

static bool checkWeightDetected(int top_sensor, int bottom_sensor) {
  if (bottom_sensor > top_sensor) {
    // Not sure what this means, but it's definitely not a weight.
    return false;
  }

  // Beyond some range we don't trust the sensors to not eg see the floor.
  if (bottom_sensor > MAX_WEIGHT_DISTANCE_MM) {
    return false;
  }
  int difference = top_sensor - bottom_sensor;
  int differencePercent = (int)(100.0f * (float)difference / (float)bottom_sensor);

  // There must be some minimum absolute and percentage difference
  return difference > MIN_WEIGHT_DISTANCE_DIFF_MM && differencePercent > MIN_WEIGHT_DISTANCE_DIFF_PERCENT;
}

// All units are in metres!
static bool detectWeight(float sensorOffsetX, float sensorOffsetY, float sensorHeading, float distanceReading) {
  // Find the position of the weight in a robot-relative frame
  float weightRelX = sensorOffsetX - distanceReading * sin(sensorHeading);
  float weightRelY = sensorOffsetY + distanceReading * cos(sensorHeading);

  float robotX, robotY;
  navGetPosition(&robotX, &robotY);

  // Find the weight position in world space, relative to the robot
  float weightX = robotX + weightRelX * cos(getIMUYaw()) - weightRelY * sin(getIMUYaw());
  float weightY = robotY + weightRelX * sin(getIMUYaw()) + weightRelY * cos(getIMUYaw());

  // Check if this weight is masked
  int cellX = (int)(weightX / STAGE_METRES_WIDTH * MAP_WIDTH);
  int cellY = (int)(weightY / STAGE_METRES_HEIGHT * MAP_HEIGHT);
  if (mapIsCollectionMasked(cellX, cellY)) {
    return false;
  }

  // From NavControl
  // Note x/y are flipped, per the coordinate system
  float targetHeading = atan2(-(weightX - robotX), weightY - robotY);

  float offsetAngle = targetHeading - getIMUYaw();
  offsetAngle *= 0.85f; // Horrible hack!

  serialPrintf("Found weight on angle: %.2f at distance %.3f m", degrees(offsetAngle), distanceReading);
  motorSelectHeading(getIMUYaw() + offsetAngle, 3500);
  forceTankTurn();

  float dx = weightX - robotX;
  float dy = weightY - robotY;
  seenDist = motorGetTotalMovementMM() + (int)(sqrt(dx * dx + dy * dy) * 1000);

  wfsmState = WFSM_SEEN;
  return true;
}

bool updateWeightFSM() {
  ToFSensorData *TOF_readings = getTOFSensorValues();
  uint16_t lower_left, lower_right, upper_left, upper_right;

  lower_left = TOF_readings->LOWER_LEFT;
  lower_right = TOF_readings->LOWER_RIGHT;

  upper_left = TOF_readings->UPPER_LEFT;
  upper_right = TOF_readings->UPPER_RIGHT;


  // serialPrintf("Lower left: %2d Upper left: %2d Lower Right: %2d Upper right: %2d", lower_left, upper_left, lower_right, upper_right);
  //  Serial.println(checkWeightDetected(upper_left, lower_left));
  // serialPrintf("Weight on left: %1d Weight on right: %1d", checkWeightDetected(upper_left, lower_left), checkWeightDetected(upper_right, lower_right));

  // int corner = drive_to_corner(TOF_readings->TOP_FRONT, TOF_readings->TOP_LEFT, TOF_readings->UPPER_RIGHT, TOF_readings->TOP_RIGHT, TOF_readings->UPPER_LEFT);

  if (upper_left < 250 || upper_right < 250) {
    loggerWrite("Upper left is %d and lower left is %d", upper_left, upper_right);
    wfsmState = WFSM_IDLE;
    serialPrintf("Wall in the way of collection");
    return false;
  }

  switch (wfsmState) {
  case WFSM_IDLE:
    {
      if (checkWeightDetected(upper_left, lower_left)) {
        if (detectWeight(-0.100f, 0.150f, -radians(SENSOR_ANGLE), lower_left / 1000.f)) {
          Serial.println("Weight on the left");
          return true;
        }
      }

      if (checkWeightDetected(upper_right, lower_right)) {
        if (detectWeight(0.100f, 0.150f, radians(SENSOR_ANGLE), lower_right / 1000.f)) {
          Serial.println("Weight on the right");
          return true;
        }
      }

      break;
    }

  case WFSM_SEEN:
    {
      navExternalDriveMark();

      if (isIPSOnMetal()) {
        wfsmState = WFSM_PROX;
        ipsStartMM = motorGetTotalMovementMM();
        serialPrintf("IPS trip");
        break;
      }

      int reversePoint = seenDist + 200;
      if (motorGetTotalMovementMM() > reversePoint) {
        // Plastic weight?
        // TODO reverse or something
        if (motorGetTotalMovementMM() > reversePoint + 400) {
          wfsmState = WFSM_IDLE;
          serialPrintf("Returned to idle, plastic weight");
        }
        motorSelectSpeeds(-3000, -3000);
      }


      break;
    }

  case WFSM_PROX: {
    if (isRevolverOffset) {
      motorImmediateStop();
      delay(200);
      run_revolver(DEGREES_60);
      delay(200);
      isRevolverOffset = false;
      break;
    }

    if (motorGetTotalMovementMM() - ipsStartMM < 100) {
      navExternalDriveMark();
      break;
    }

    motorSelectHeading(getIMUYaw() + radians(15), 3500);
    motorForceTurnOnSpot();
    wfsmState = WFSM_WIGGLE1;
    break;
  }
  case WFSM_WIGGLE1: {
    if (motorGetTotalMovementMM() - ipsStartMM < 120) {
      navExternalDriveMark();
      break;
    }

    motorSelectHeading(getIMUYaw() - radians(30), 3500);
    motorForceTurnOnSpot();
    wfsmState = WFSM_WIGGLE2;
    break;
  }

  case WFSM_WIGGLE2:
    {
      if (motorGetTotalMovementMM() - ipsStartMM < 140) {
        navExternalDriveMark();
        break;
      }

      motorSelectHeading(getIMUYaw() + radians(15), 3500);
      motorForceTurnOnSpot();
      wfsmState = WFSM_WIGGLE3;
      break;
    }

  case WFSM_WIGGLE3:
    {
      // This is the time since the IPS sensor was tripped.
      // Wait for awhile
      if (motorGetTotalMovementMM() - ipsStartMM < 280) {
        break;
      }

      serialPrintf("Stop");

      // Cut the motor PWMs, as we don't want to move while collecting
      // the weight as the navigation system won't be running.
      motorImmediateStop();

      delay(500);

      serialPrintf("Running");

      // Ensure the revolver ends up rotates 60 degrees once full
      if (weights_onboard < 2) {
        run_revolver(DEGREES_120);
        isRevolverOffset = false;
      } else {
        run_revolver(DEGREES_60);
        isRevolverOffset = true;
      }

      serialPrintf("Done");
      weights_onboard += 1; // Record increase of weights onboard

      // Go back to the start of the FSM
      wfsmState = WFSM_IDLE;
      motorSelectSpeeds(0, 0);

      break;
    }
  }

  return wfsmState != WFSM_IDLE;
}

void updateMasterFSM() {
  switch (masterState) {
  case MFSM_WAITING_TO_START:
    {
      if (!hasSampledSide && millis() > 2000) {
        isHomeOnRight = getTOFSensorValues()->TOP_RIGHT < getTOFSensorValues()->TOP_LEFT;
        serialPrintf("On right side? %d", isHomeOnRight);
        hasSampledSide = true;
        navInit(isHomeOnRight);
      }

      if (isStartPressed()) {
        serialPrintf("Start pressed!");
        masterState = MFSM_ROAMING;
      }
      break;
    }
  case MFSM_ROAMING:
    {
      if (updateWeightFSM()) {
        // We've seen a weight, we're trying to pick it up
      } else {
        navDrive(true);
      }

      if (weights_onboard == 3) {
        masterState = MFSM_RETURNING;
        navReturnToBase();
      }
      break;
    }
  case MFSM_RETURNING:
    {
      if (navDrive(false)) {
        return;
      }

      int offset = 25;
      if (isHomeOnRight)
        offset *= -1;
      float targetHeading = radians(180 - offset);
      motorSelectHeading(targetHeading, 0);
      motorForceTurnOnSpot();
      if (abs(angleDifference(getIMUYaw(), targetHeading)) > radians(5)) {
        return;
      }

      masterState = MFSM_DRIVE_TO_CORNER;
      break;
    }
  case MFSM_DRIVE_TO_CORNER:
    {
      ToFSensorData *TOF_readings = getTOFSensorValues();
      bool running = drive_to_corner(TOF_readings->TOP_FRONT, TOF_readings->TOP_LEFT, TOF_readings->UPPER_RIGHT, TOF_readings->TOP_RIGHT, TOF_readings->UPPER_LEFT);
      if (!running) {
        masterState = MFSM_DROPPING;
      }
      break;
    }
  case MFSM_DROPPING:
    if (!weight_dropoff()) {
      masterState = MFSM_ROAMING;
      navReturnToBase();
    }
    break;
  }
}
