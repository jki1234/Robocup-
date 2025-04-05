// Uncomment to enable all the debug prints:
// #define DEBUG

#include "Sensors.h"
#include "Utils.h"

#include <Wire.h>
#include "src/Adafruit_BNO055/Adafruit_BNO055.h"
#include "src/SX1509_IO_Expander/SparkFunSX1509.h"
#include "src/VL53L0X/VL53L0X.h"
#include "src/VL53L1X/VL53L1X.h"

const int PROX_SENSOR_PIN = 8;

const byte SX1509_ADDRESS_XSHUT = 0x3F;
const byte SX1509_ADDRESS_AIO = 0x3E;
#define VL53L0X_ADDRESS_START 0x30
#define VL53L1X_ADDRESS_START 0x35
#define ECHO_DURATION 1000


// Note these must match up with the order of the values defined in ToFSensorData
const uint8_t shortRangePins[] = {0}; // XSHUT pins
const uint8_t longRangePins[] = {1, 6, 7, 3, 2, 5}; // XSHUT pins
const uint8_t sensorCountShortRange = ARRAY_SIZE(shortRangePins);
const uint8_t sensorCountLongRange = ARRAY_SIZE(longRangePins);


static union tofDistances_t {
  uint16_t array[sensorCountShortRange + sensorCountShortRange];
  ToFSensorData dataStruct;
} tofDistances = {};

static float imuYaw = 0;
static float imuPitch = 0;
static float imuBank = 0;

static bool inductiveSensorState = false;

SX1509 io(&Wire); // Create an SX1509 object to be used throughout
SX1509 io2(&Wire1); // U1 on schematic
VL53L0X sensorsL0[sensorCountShortRange];
VL53L1X sensorsL1[sensorCountLongRange];

Adafruit_BNO055 theIMU = Adafruit_BNO055(55, 0x28, &Wire);

imu::Quaternion initialRotation;
long microsecondsToCentimetres(long);

// The library doesn't have this :(
// I copied the maths of wikipedia for it.
imu::Quaternion quatInverse(const imu::Quaternion &q) {
  double scale = 1 / (q.w() * q.w() + q.x() * q.x() + q.y() * q.y() + q.z() * q.z());
  return imu::Quaternion(scale * q.w(), -scale * q.x(), -scale * q.y(), -scale * q.z());
}

void DebugSearchSensors() {
  io.begin(SX1509_ADDRESS_XSHUT);

  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  for (uint8_t i = 0; i < 8; i++) {
    io.pinMode(i, OUTPUT);
    io.digitalWrite(i, LOW);
  }

  for (uint8_t i = 0; i < 8; i++) {
    io.digitalWrite(i, HIGH);
    delay(10);

    sensorsL0[0].setTimeout(500);
    serialPrintf("Sensor SR: %d init %d", i, sensorsL0[0].init());

    io.digitalWrite(i, LOW);
    delay(100);
    io.digitalWrite(i, HIGH);
    delay(10);

    sensorsL1[0].setTimeout(500);
    serialPrintf("Sensor LR: %d init %d", i, sensorsL1[0].init());

    io.digitalWrite(i, LOW);
  }
  while (true)
    ;
}

void SetupSensors() {
  pinMode(START_BUTTON, INPUT); // Intialise the start button
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C
  Wire1.begin();
  Wire1.setClock(400000); // use 400 kHz I2C

  if (!io.begin(SX1509_ADDRESS_XSHUT))
    serialPrintf("Failed to init xshut SX1509");
  if (!io2.begin(SX1509_ADDRESS_AIO))
    serialPrintf("Failed to init AIO SX1509");

  io2.pinMode(8, INPUT);

  // Disable/reset all sensors by driving their XSHUT pins low.
  // Do this regardless of the shortRangePins, as if we don't specify one of
  // the sensors we'd otherwise leave it on, and that'd cause the init() to fail.
  // This is really annoying to debug (ask me how I know).
  for (uint8_t i = 0; i < 8; i++) {
    io.pinMode(i, OUTPUT);
    io.digitalWrite(i, LOW);
  }
  delay(50);

  if (!theIMU.begin()) {
    serialPrintf("No BNO055 detected");
    while (true)
      ;
  }

  // L0 Enable, initialize, and start each sensor, one by one.
  for (uint8_t i = 0; i < sensorCountShortRange; i++) {
    // Stop driving this sensor's XSHUT low. This should allow the carrier
    // board to pull it high. (We do NOT want to drive XSHUT high since it is
    // not level shifted.) Then wait a bit for the sensor to start up.
    // pinMode(xshutPins[i], INPUT);
    io.digitalWrite(shortRangePins[i], HIGH);
    delay(10);

    sensorsL0[i].setTimeout(500);
    if (!sensorsL0[i].init()) {
      serialPrintf("Failed to detect and initialize sensor L0 %d", i);
      while (true)
        ; // TODO clean this up
    }

    // Each sensor must have its address changed to a unique value other than
    // the default of 0x29 (except for the last one, which could be left at
    // the default). To make it simple, we'll just count up from 0x2A.
    sensorsL0[i].setAddress(VL53L0X_ADDRESS_START + i);

    sensorsL0[i].startContinuous(50);
  }

  for (uint8_t i = 0; i < sensorCountLongRange; i++) {
    io.digitalWrite(longRangePins[i], HIGH);
    delay(10);

    sensorsL1[i].setTimeout(500);
    if (!sensorsL1[i].init()) {
      serialPrintf("Failed to detect and initialize sensor long range %d", i);
      while (true)
        ; // TODO clean this up
    }

    // Each sensor must have its address changed to a unique value other than
    // the default of 0x29 (except for the last one, which could be left at
    // the default). To make it simple, we'll just count up from 0x2A.
    sensorsL1[i].setAddress(VL53L1X_ADDRESS_START + i);

    // Narrow down the beam, so the front sensors don't see the ground.
    sensorsL1[i].setROISize(4, 16);

    sensorsL1[i].startContinuous(50);
  }
}

void UpdateSensors() {
  // Check if the ToF sensors have new data, and only read if they do.
  // Otherwise, the sensor libraries will wait until they get data, blocking our main loop.
  // TODO add our own timeout system, in case a sensor never reports valid data.
  for (uint8_t i = 0; i < sensorCountShortRange; i++) {
    // Copied from readRangeContinuousMillimeters, which waits until this is set.
    if ((sensorsL0[i].readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) == 0)
      continue;

    uint16_t range = sensorsL0[i].readRangeContinuousMillimeters();
    tofDistances.array[i] = range;
  }
  for (uint8_t i = 0; i < sensorCountLongRange; i++) {
    if (!sensorsL1[i].dataReady())
      continue;

    uint16_t range = sensorsL1[i].readRangeContinuousMillimeters();
    tofDistances.array[i + sensorCountShortRange] = range;
  }

  DEBUG_PRINTF(
      "TOF Sensors: top %4d %4d %4d   upper %4d %4d   lower %4d %4d",
      // Front
      tofDistances.dataStruct.TOP_LEFT, tofDistances.dataStruct.TOP_FRONT, tofDistances.dataStruct.TOP_RIGHT,
      // Lower
      tofDistances.dataStruct.UPPER_LEFT, tofDistances.dataStruct.UPPER_RIGHT,
      // Upper
      tofDistances.dataStruct.LOWER_LEFT, tofDistances.dataStruct.LOWER_RIGHT
  );

  imu::Quaternion sample = theIMU.getQuat();
  if (initialRotation.w() == 0 && initialRotation.x() == 0 && initialRotation.y() == 0 && initialRotation.z() == 0) {
    initialRotation = sample;

    // The IMU outputs all-zero quaternions until it's working, which make NaNs pop up!
    return;
  }

  imu::Quaternion quat = quatInverse(initialRotation) * sample;

  imu::Vector<3> forwardVector = quat.rotateVector(imu::Vector<3>(0, 1, 0));
  imu::Vector<3> sideVector = quat.rotateVector(imu::Vector<3>(1, 0, 0));
  imuPitch = (float)asin(forwardVector.z());
  imuBank = (float)asin(sideVector.z());
  imuYaw = (float)atan2(-forwardVector.x(), forwardVector.y());

  // DEBUG_PRINTF("IMU fused state: w=%f  x=%f  y=%f  z=%f    pitch=%f bank=%f yaw=%f", quat.w(), quat.x(), quat.y(), quat.z(), degrees(imuPitch), degrees(imuBank), degrees(imuYaw));


  // The inductive sensor output is inverted, so invert it back again.
  inductiveSensorState = !io2.digitalRead(PROX_SENSOR_PIN);
  // DEBUG_PRINTF("Inductive sensor output: %d", inductiveSensor);
}

long microsecondsToCentimetres(long microseconds) {
  return microseconds / 29 / 2;
}

/**
 * Get the time-of-flight sensor readings.
 *
 * This never causes I2C traffic.
 */
ToFSensorData *getTOFSensorValues() {
  return &tofDistances.dataStruct;
}

/*
 * Get the yaw of the robot in degrees.
 *
 * This ranges from -180 to 180. Positive values are counter-clockwise, when the robot
 * is viewed from above.
 */
float getIMUYaw() {
  return imuYaw;
}

/**
 * Get the robot's pitch in degrees, as measured by the IMU.
 *
 * Zero means the robot is level, positive means the front has been lifted (eg if it's
 * driving up a ramp), negative values means the robot's rear has been lifted.
 */
float getIMUPitch() {
  return imuPitch;
}

/**
 * Get the robot's bank angle in degrees, as measured by the IMU.
 *
 * Zero means the robot is level, positive means it's banking to the left, negative
 * means it's banking to the right.
 */
float getIMUBank() {
  return imuBank;
}

float angleDifference(float a, float b) {
  float result = fmodf(a - b, PI * 2);
  if (result < -PI) {
    result += PI * 2;
  }
  if (result > PI) {
    result -= PI * 2;
  }
  return result;
}

bool isIPSOnMetal() {
  return inductiveSensorState;
}

bool isStartPressed() {
  return digitalRead(START_BUTTON);
}
