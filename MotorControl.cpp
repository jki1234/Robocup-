// #define DEBUG

#include "MotorControl.h"
#include "PI.h"
#include "Sensors.h"
#include "Utils.h"
#include "wiring.h"

void set_motor(int motor, int motor_speed_percentage);

Servo myservoLeft, myservoRight; // create servo object to control a servo

const float KP = 0.02;
const float KI = 0.1;
const float MAX_KI_TERM = 100; // Max contribution from the integral term, in percent
const float MAX_SLEW_RATE = 200; // In percent per second

const float TURN_ON_SPOT_START_ANGLE = 40;
const float TURN_ON_SPOT_END_ANGLE = 2;
const float TURN_ON_SPOT_SPEED = 4000;
const float SPEED_PER_DEGREE_OFFSET = 40;

DeltaTimeMonitor motorUpdateTimer(0.5);
long lastMotorUpdateMicros = 0;

RateLimiter leftRateLimiter(MAX_SLEW_RATE, 100, 0);
PIController leftSpeedControl(KP, KI, MAX_KI_TERM, -MAX_KI_TERM);
RateLimiter rightRateLimiter(MAX_SLEW_RATE, 100, 0);
PIController rightSpeedControl(KP, KI, MAX_KI_TERM, -MAX_KI_TERM);

static void leftEncoderISR();
static void rightEncoderISR();

MotorController leftMotor(0, 2, 3, true, leftEncoderISR);
MotorController rightMotor(1, 4, 5, false, rightEncoderISR);

IntervalTimer motorTimer;
void motorTimerISR();

int motorMode = MOTOR_MODE_SPEEDS;
float targetHeading = 0;
float targetSpeed = 0;
float leftTargetSpeed = 0;
float rightTargetSpeed = 0;
bool turningOnSpot = false;
volatile bool immediateStopped = false;

// Add movement into a float, and periodically move it into the MM int register.
volatile int totalMovementMM = 0;
float totalMovementAccumulator = 0; // in metres

void motor_init() {
  // initialise drive motor
  myservoRight.attach(1);
  myservoLeft.attach(0);

  // Set up the timer to trigger the motor control interrupt
  // This means the motor will work properly even if the rest of the code is looping slowly.
  motorTimer.begin(motorTimerISR, 10'000);
}

void motorSelectHeading(float heading, int speed) {
  motorMode = MOTOR_MODE_HEADING;
  targetHeading = heading;
  targetSpeed = (float)speed;
}

void motorSelectSpeeds(int leftSpeed, int rightSpeed) {
  motorMode = MOTOR_MODE_SPEEDS;
  leftTargetSpeed = (float)leftSpeed;
  rightTargetSpeed = (float)rightSpeed;
}

void forceTankTurn() {
  turningOnSpot = true;
}

void motorUpdate() {
  immediateStopped = false;

  // Do the IMU-based control here. We could do it in the ISR, but we don't need to update it
  // nearly as frequently as the motor inputs since we're getting new IMU readings much slower too.
  if (motorMode == MOTOR_MODE_HEADING) {
    float yaw = getIMUYaw();
    float delta = angleDifference(targetHeading, yaw);

    // If we need to make a big turn, rotate on the spot. Otherwise, we can rotate as we're driving.
    // We have some hysterisis here to avoid switching back and forth quickly.
    if (abs(delta) > radians(TURN_ON_SPOT_START_ANGLE)) {
      turningOnSpot = true;
    }
    if (abs(delta) < radians(TURN_ON_SPOT_END_ANGLE)) {
      turningOnSpot = false;
    }

    bool ccw = delta > 0; // If true, we need to turn left. Otherwise, turn right.
    float positiveLeft = ccw ? 1 : -1;

    if (turningOnSpot) {
      leftTargetSpeed = -positiveLeft * TURN_ON_SPOT_SPEED;
      rightTargetSpeed = positiveLeft * TURN_ON_SPOT_SPEED;
    } else {
      float speedDelta = SPEED_PER_DEGREE_OFFSET * (float)degrees(delta);
      leftTargetSpeed = targetSpeed - speedDelta;
      rightTargetSpeed = targetSpeed + speedDelta;
    }
  }
}

bool motorIsTurningOnSpot() {
  return turningOnSpot;
}

void motorForceTurnOnSpot() {
  turningOnSpot = true;
}

int motorGetTotalMovementMM() {
  return totalMovementMM;
}

// ISR safe
// Speed indicates direction: +ve forward, -ve backwards
void set_motor(int motor, int motor_speed_percentage) {
  int motor_speed;

  // Reverse the direction on the left track, since the motor is mounted
  // the other way around there.
  if (motor == MOTOR_LEFT) {
    motor_speed_percentage *= -1;
  }

  if (motor_speed_percentage > 0)
    motor_speed = map(motor_speed_percentage, 0, 100, 1500, 1950);
  else
    motor_speed = map(-motor_speed_percentage, 0, 100, 1500, 1050);


  if (motor == MOTOR_LEFT)
    myservoLeft.writeMicroseconds(motor_speed);
  else if (motor == MOTOR_RIGHT)
    myservoRight.writeMicroseconds(motor_speed);
}

void motorImmediateStop() {
  immediateStopped = true;
  set_motor(MOTOR_LEFT, 0);
  set_motor(MOTOR_RIGHT, 0);
}

void motorTimerISR() {
  float dt = motorUpdateTimer.getDeltaTime();

  if (immediateStopped)
    return;

  float v = leftSpeedControl.update(dt, abs(leftTargetSpeed), (float)leftMotor.getEncoderSpeed());
  v = leftRateLimiter.update(dt, v);
  if (leftTargetSpeed < 0)
    v *= -1;
  set_motor(MOTOR_LEFT, (int)v);

  float leftV = v;

  v = rightSpeedControl.update(dt, abs(rightTargetSpeed), (float)rightMotor.getEncoderSpeed());
  v = rightRateLimiter.update(dt, v);
  if (rightTargetSpeed < 0)
    v *= -1;
  set_motor(MOTOR_RIGHT, (int)v);

  // Might not be safe in an ISR, but it seems to work.
  DEBUG_PRINTF("%d %f %d %f %f", leftMotor.getEncoderSpeed(), leftV, rightMotor.getEncoderSpeed(), v, dt);
}

static void leftEncoderISR() {
  leftMotor.handleISR();
}
static void rightEncoderISR() {
  rightMotor.handleISR();
}


MotorController::MotorController(int servoPin, int encoderPinA, int encoderPinB, bool reversed, void (*encoderISR)()) :
    m_servoPin(servoPin), m_encoderPinA(encoderPinA), m_encoderPinB(encoderPinB), m_reversed(reversed) {

  pinMode(m_encoderPinA, INPUT);
  pinMode(m_encoderPinB, INPUT);

  // Set up an interrupt for each encoder
  attachInterrupt(digitalPinToInterrupt(m_encoderPinA), encoderISR, CHANGE);
}

void MotorController::handleISR() {
  unsigned int lastEP = m_encoderPos;

  // Test transition
  bool encoderA = digitalRead(m_encoderPinA) == HIGH;
  // and adjust counter + if A leads B
  m_encoderPos += ((encoderA != m_encoderB) ^ m_reversed) ? +1 : -1;

  m_encoderB = digitalRead(m_encoderPinB) == HIGH;
  // and adjust counter + if B follows A
  m_encoderPos += ((encoderA == m_encoderB) ^ m_reversed) ? +1 : -1;


  // Accumulate the total movement
  // Cast it to a signed int to deal with overflow/underflow
  int movement = abs((int)(m_encoderPos - lastEP));

  // Divide by 2 since this is only one of the two wheels
  if (!turningOnSpot) {
    totalMovementAccumulator += movement / 2.0f * ENCODER_TO_M;
  }

  // Once we've driven forwards by a mm, store that out.
  int mm = (int)(totalMovementAccumulator * 1000);
  totalMovementAccumulator -= mm / 1000.0f;
  totalMovementMM += mm;


  unsigned long now = micros();
  long elapsed = (long)(now - m_lastEncoderMicros);
  m_lastEncoderMicros = now;

  // Handle micros() overflow
  if (elapsed < 0) {
    return;
  }

  // Anything longer than 100ms means the robot was *definitely* stopped before
  // We also need this so our cast-to-int is valid.
  if (elapsed > 100'000) {
    return;
  }

  int firBufSize = sizeof(m_encoderInverseSpeedFIR) / sizeof(m_encoderInverseSpeedFIR[0]);
  m_encoderInverseSpeedFIR[m_encoderInverseSpeedFIRPos++] = (int)elapsed;
  m_encoderInverseSpeedFIRPos %= firBufSize;

  int sum = 0;
  for (int i = 0; i < firBufSize; i++) {
    sum += m_encoderInverseSpeedFIR[i];
  }

  // Something went horribly wrong, wait for the bad data to cycle out of the buffer
  if (sum < 0) {
    return;
  }

  // 2x: We're measuring the changes to encoder output A, so encoder output B is changing
  //     as well and we're not measuring that. Thus the measured speed is half the true value.
  // 1.000.000: Microseconds per second.
  // /sum: The average inverse speed, in micros/encoder A ticks times the FIR buffer size.
  // *firBufSize: Cancel out the firBufSize component of the sum.
  // This order is picked (multiply before divide) to avoid unnecessary rounding.
  m_encoderSpeed = 2 * 1'000'000 * firBufSize / sum;
}
