#pragma once

#include <Arduino.h>
#include <Servo.h>
#include <stdint.h>

/**
 * How many meters there are in one encoder tick.
 */
constexpr float ENCODER_TO_M = 7.136e-05;

enum { MOTOR_LEFT, MOTOR_RIGHT };
enum { MOTOR_MODE_HEADING, MOTOR_MODE_SPEEDS };

void motor_init();

void motorUpdate();

/**
 * Drive the robot on a given heading in radians.
 *
 * This tries to match getIMUYaw to the given heading.
 *
 * Speed is the target speed to move at, in encoder ticks per second.
 */
void motorSelectHeading(float heading, int speed);

/**
 * Set the motors to a given pair of speeds, in encoder ticks per second.
 */
void motorSelectSpeeds(int leftSpeed, int rightSpeed);

void forceTankTurn();

/**
 * ONLY use this when driving the stepper motor.
 *
 * This stops taking effect next time motorUpdate is called.
 *
 * You should normally call motorSelectSpeeds(0, 0) to stop the robot.
 */
void motorImmediateStop();

/**
 * Returns true if the robot is turning on the spot, due to motorSelectHeading.
 */
bool motorIsTurningOnSpot();

void motorForceTurnOnSpot();

/**
 * Get the total distance the robot has moved in millimetres since startup.
 *
 * This is monotonic: it only ever increases. Both forwards and backwards movement
 * increases it's value.
 *
 * Note this won't be exactly accurate if the robot is doing a slow (arc-style) turn,
 * it's mostly intended for when the robot is driving in a straight line.
 *
 * Turning on the spot (tank turns) doesn't contribute anything to this value.
 */
int motorGetTotalMovementMM();

class MotorController {
public:
  MotorController(int servoPin, int encoderPinA, int encoderPinB, bool reversed, void (*encoderISR)());

  /**
   * Should be called by the encoderISR function passed to the constructor.
   */
  void handleISR();

  unsigned int getEncoderPos() {
    return m_encoderPos;
  }

  int getEncoderSpeed() {
    return m_encoderSpeed;
  }

private:
  int m_servoPin;
  int m_encoderPinA;
  int m_encoderPinB;
  bool m_reversed;

  volatile unsigned int m_encoderPos = 0;
  bool m_encoderB = false;

  volatile unsigned int m_encoderSpeed = 0; // Encoder ticks per second

  unsigned long m_lastEncoderMicros = 0;
  int m_encoderInverseSpeedFIR[10] = {};
  int m_encoderInverseSpeedFIRPos = 0;
};

extern MotorController leftMotor;
extern MotorController rightMotor;
