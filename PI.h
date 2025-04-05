#pragma once

class DeltaTimeMonitor {
public:
  DeltaTimeMonitor(float maxStepSeconds);

  /**
   * Get the time in seconds since this function was last called.
   *
   * If update is false, this only reads the current value without resetting
   * the delta-time back to zero.
   *
   * The result is set to zero if the delta-time is more than maxStepSeconds,
   * as passed to the constructor. This is to guard against massive delta-times
   * mucking up control logic (eg the integral term of a PID controller) if
   * there's an unusual delay in the control system.
   */
  float getDeltaTime(bool update = true);

private:
  float m_maxStepSeconds;

  unsigned long m_lastMicros = 0;
};

/**
 * A utility class that implements rate limiting and output clamping.
 *
 * This is where we limit the rate of change of some output, to avoid
 * sudden and sharp output changes.
 */
class RateLimiter {
public:
  /**
   * Create a new rate limiter with symmetrical slew rates.
   *
   * The slew rate is in units per second.
   */
  RateLimiter(float slewRate, float maxOutput, float minOutput);

  /**
   * Create a new rate limiter.
   *
   * The max/min slew rates should be +ve/-ve respectively, in units per second.
   */
  RateLimiter(float maxSlewRate, float minSlewRate, float maxOutput, float minOutput);

  float update(float dt, float newValue);

  void stepTo(float newValue);

private:
  float m_maxSlewRate, m_minSlewRate;
  float m_maxOutput, m_minOutput;
  float m_last = 0;
};

class PIController {
public:
  PIController(float kp, float ki, float intMax, float intMin);

  float update(float deltaTime, float setpoint, float value);

private:
  float m_kp, m_ki;
  float m_intMax, m_intMin;
  float m_int = 0;
};
