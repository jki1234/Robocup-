#define DEBUG

#include "NavControl.h"

#include "Collection.h"
#include "MapParser.h"
#include "MotorControl.h"
#include "SDCard.h"
#include "Sensors.h"
#include "Utils.h"

// The minimum distance to the next target we select
static constexpr float NEXT_TARGET_MIN_DIST = 0.100;

// The minimum distance to the current target point before
// we'll switch over to the next one.
static constexpr float MIN_SWITCH_TARGET_DIST = 0.100;

static unsigned int lastLeftPos = 0, lastRightPos = 0;

static float robotX, robotY;
static bool isHomeOnRight;

static bool hasTargetWeight = false;
static MapCell targetWeightPos = 0;

static bool hasTargetPos = false;
static bool atLastPos = false;
static float targetX = 0;
static float targetY = 0;
static MapCell targetGridPos = 0;

// The previous distance to the target position.
// This is used to drive closer than MIN_SWITCH_TARGET_DIST, but
// switch as soon as we start moving away.
static float lastDistToTarget = 0;

static MapCell findStartPos();

static void updateTarget();

// #define serialPrintf loggerWrite

void navInit(bool isBaseOnRight) {
  float margin = 0.100;
  robotX = 0.15f + margin;
  robotY = 0.18f + margin;

  isHomeOnRight = isBaseOnRight;

  if (isBaseOnRight) {
    robotX = STAGE_METRES_WIDTH - robotX;
  }
}

void navUpdate() {
  // This is technically racy with interrupts, but only for +/- one encoder tick.
  // (no tearing because these are 32-bit values on a 32-bit CPU)
  unsigned int leftPos = leftMotor.getEncoderPos();
  unsigned int rightPos = rightMotor.getEncoderPos();

  float yaw = getIMUYaw();

  // +ve is forward for both motors - flipping one of them is handled by MotorControl.cpp
  int deltaLeft = (int)(leftPos - lastLeftPos);
  int deltaRight = (int)(rightPos - lastRightPos);
  lastLeftPos = leftPos;
  lastRightPos = rightPos;

  // divide by 2 to find the average
  float movement = (deltaLeft + deltaRight) / 2.0f * ENCODER_TO_M;

  // This is kinda horrible, but just ignore any movement if we're turning on-the-spot.
  // Otherwise small differences in the belt speed that don't result in any physical
  // forward movement lead to a lot of drift.
  if (motorIsTurningOnSpot()) {
    movement = 0;
  }

  // A heading of zero is facing down the long axis of the arena
  // 0,0 is in the green corner (left side), +ve x is towards the blue corner
  // This assumes we're not driving up a ramp, TODO deal with that
  robotX += movement * -sin(yaw); // +ve yaw is CCW but +ve x is in the opposite direction
  robotY += movement * cos(yaw);
}

bool navDrive(bool pickNewTargets) {
  if (!hasTargetPos) {
    if (!pickNewTargets) {
      return false;
    }

    updateTarget();

    // Don't trigger anything immediately
    lastDistToTarget = 1e9;

    // Force a tank turn whenever we select a new point, to avoid large gradual turns.
    motorForceTurnOnSpot();
  }

  // Are there no weights left?
  if (!hasTargetPos) {
    // TODO pick random point
    return false;
  }

  // Drive towards the target

  float targetHeading = atan2(-(targetX - robotX), targetY - robotY); // Note x/y are flipped, per the coordinate system
  float targetDist = sqrt(pow(targetX - robotX, 2) + pow(targetY - robotY, 2));

  if (targetDist >= MIN_SWITCH_TARGET_DIST) {
    // Drive towards the target
    motorSelectHeading(targetHeading, 3500);
  } else {
    // We're close enough to the target waypoint that we can accept the next one.
    // Only switch to the next target when we've stopped getting closer though - if we're
    // still driving towards the waypoint, we can get it a bit more accurate very easily.
    if (targetDist > lastDistToTarget) {
      motorSelectSpeeds(0, 0);
      markWeightAsCollected(targetGridPos);
      serialPrintf("Reached point at %d %d", mapExtractX(targetGridPos), mapExtractY(targetGridPos));
      hasTargetPos = false;

      // If this was the last point in the pathfinding chain, pick a new weight next turn.
      if (atLastPos) {
        hasTargetWeight = false;
      }

      return true;
    }

    // Continue on the current heading, don't turn around to try and get slightly
    // closer as we can get stuck spinning around.
    motorSelectHeading(getIMUYaw(), 3500);
  }

  lastDistToTarget = targetDist;

  // DEBUG_PRINTF("pos: %f %f    tgt %f %f    hdg %f   dist %f     %d %d", robotX, robotY, targetX, targetY, targetHeading, targetDist, leftPos, rightPos);
  // serialPrintf("%f,%f,%f,%f,%f,%f,%f,%f\n", micros() / 1.0e6f, robotX, robotY, targetX, targetY, targetDist, targetHeading, getIMUYaw());

  return true;
}

void navGetPosition(float *outX, float *outY) {
  *outX = robotX;
  *outY = robotY;
}

void navReturnToBase() {
  hasTargetPos = false;
  hasTargetWeight = false;

  updateTarget();

  // Don't trigger anything immediately
  lastDistToTarget = 1e9;

  // Force a tank turn whenever we select a new point, to avoid large gradual turns.
  motorForceTurnOnSpot();
}

static MapCell findStartPos() {
  int baseX = (int)(robotX / STAGE_METRES_WIDTH * MAP_WIDTH);
  int baseY = (int)(robotY / STAGE_METRES_HEIGHT * MAP_HEIGHT);

  // If we start inside a wall, find a nearby cell that's not obstructed
  if (!mapHasObstruction(baseX, baseY))
    return mapPackPos(baseX, baseY);

  for (int i = 1; i < 200; i++) {
    if (!mapHasObstruction(baseX + i, baseY))
      return mapPackPos(baseX + i, baseY);
    if (!mapHasObstruction(baseX - i, baseY))
      return mapPackPos(baseX - i, baseY);

    if (!mapHasObstruction(baseX, baseY + i))
      return mapPackPos(baseX, baseY + i);
    if (!mapHasObstruction(baseX, baseY - i))
      return mapPackPos(baseX, baseY - i);

    if (!mapHasObstruction(baseX + i, baseY + i))
      return mapPackPos(baseX + i, baseY + i);
    if (!mapHasObstruction(baseX - i, baseY - i))
      return mapPackPos(baseX - i, baseY - i);

    if (!mapHasObstruction(baseX + i, baseY - i))
      return mapPackPos(baseX + i, baseY - i);
    if (!mapHasObstruction(baseX - i, baseY + i))
      return mapPackPos(baseX - i, baseY + i);
  }

  // WTF are we supposed to do?
  return mapPackPos(0, 0);
}

void pickNextWeightIfNecessary(MapCell startCell) {
  if (hasTargetWeight)
    return;

  if (weights_onboard == 3) {
    int x = 250 / 20;
    if (isHomeOnRight) {
      x = MAP_WIDTH - x;
    }
    targetWeightPos = mapPackPos(x, 250 / 20);
    hasTargetWeight = true;
    return;
  }

  serialPrintf("Finding path to new weight");
  int cost = mapFindNearestWeight(startCell, &targetWeightPos);
  serialPrintf("done.");
  if (cost == -1) {
    return; // No path found
  }
  hasTargetWeight = true;
}

void updateTarget() {
  // In case we fail early
  hasTargetPos = false;

  int mapX = (int)(robotX / STAGE_METRES_WIDTH * MAP_WIDTH);
  int mapY = (int)(robotY / STAGE_METRES_HEIGHT * MAP_HEIGHT);

  MapCell startCell = findStartPos();
  // MapCell endCell = mapPackPos(MAP_WIDTH / 2, MAP_HEIGHT - 1);

  pickNextWeightIfNecessary(startCell);

  if (!hasTargetWeight) {
    // TODO do something useful here, go home or fall back to the random walk PF algorithm
    serialPrintf("No path to any weight! %f %f", robotX, robotY);
    return;
  }

  MapCell endCell = targetWeightPos;

  int weight = mapFindPath(startCell, endCell);
  if (weight == -1) {
    serialPrintf("No path! %d %d   %f %f", mapX, mapY, robotX, robotY);
    return; // No path found
  }

  // Find the first cell we can reach that's at least a certain distance away
  MapCell prev = endCell;
  MapCell cell;
  while (true) {
    cell = mapPathNextSmooth(prev);

    // If we've hit the start of the path, stop iterating.
    if (cell == prev) {
      break;
    }

    // Convert the waypoint into a x,y position in metres
    float metresX = 1.0f * (float)mapExtractX(cell) / MAP_WIDTH * STAGE_METRES_WIDTH;
    float metresY = 1.0f * (float)mapExtractY(cell) / MAP_HEIGHT * STAGE_METRES_HEIGHT;

    // Keep iterating till we get too close to the robot
    float dx = metresX - robotX;
    float dy = metresY - robotY;
    float dist = (float)sqrt(dx * dx + dy * dy); // NOLINT(*-type-promotion-in-math-fn)

    serialPrintf("Follow path %f %f (%f)    ", metresX, metresY, dist);

    // Have we found a point that's too close to the robot?
    // If so, stop and use the previous cell
    if (dist < NEXT_TARGET_MIN_DIST)
      break;

    prev = cell;
  }

  // Use the last cell, which is further away.
  targetX = 1.0f * (float)mapExtractX(prev) / MAP_WIDTH * STAGE_METRES_WIDTH;
  targetY = 1.0f * (float)mapExtractY(prev) / MAP_HEIGHT * STAGE_METRES_HEIGHT;
  hasTargetPos = true;
  targetGridPos = cell;
  serialPrintf("Picked next: %f %f\n", targetX, targetY);

  // If we've hit the end, then drive to that point, regardless of whether we're nearby.
  // This forces the robot to turn towards it, if it's not already.
  if (prev == endCell) {
    // Pick a new weight next turn.
    // Overwrite targetGridPos to the weight position, in case there's a bug
    // in the pathfinder. In this case, we'll at least continue even if it means
    // we miss a weight.
    atLastPos = true;
    targetGridPos = targetWeightPos;

    serialPrintf("Selecting end point.\n");
  }
}

void navExternalDriveMark() {
  if (!hasTargetPos) {
    return;
  }

  // If we get close enough to the weight position while we're driving, mark it as collected.
  float targetDist = sqrt(pow(targetX - robotX, 2) + pow(targetY - robotY, 2));

  if (targetDist >= MIN_SWITCH_TARGET_DIST) {
    return;
  }

  markWeightAsCollected(targetGridPos);
  serialPrintf("Reached point at %d %d while driving", mapExtractX(targetGridPos), mapExtractY(targetGridPos));
  hasTargetPos = false;

  // If this was the last point in the pathfinding chain, pick a new weight next turn.
  if (atLastPos) {
    hasTargetWeight = false;
  }
}
