#include "PI.h"

#include <Arduino.h>

DeltaTimeMonitor::DeltaTimeMonitor(float maxStepSeconds) : m_maxStepSeconds(maxStepSeconds) {
}

float DeltaTimeMonitor::getDeltaTime(bool update) {
  unsigned long now = micros();
  float dt = (float)(long)(now - m_lastMicros) / 1'000'000.f;

  if (update) {
    m_lastMicros = now;
  }

  // Don't integrate if we have an unrealistic delta-time - this suggests that something
  // weird has happened, maybe an overflow.
  if (dt < 0) {
    dt = 0;
  }
  if (dt > m_maxStepSeconds) {
    dt = 0;
  }

  return dt;
}

RateLimiter::RateLimiter(float slewRate, float maxOutput, float minOutput) : RateLimiter(slewRate, -slewRate, maxOutput, minOutput) {
}

RateLimiter::RateLimiter(float maxSlewRate, float minSlewRate, float maxOutput, float minOutput) :
    m_maxSlewRate(maxSlewRate), m_minSlewRate(minSlewRate), m_maxOutput(maxOutput), m_minOutput(minOutput) {
}

float RateLimiter::update(float dt, float newValue) {
  newValue = min(m_maxOutput, max(m_minOutput, newValue));

  float delta = newValue - m_last;
  delta = min(m_maxSlewRate * dt, max(m_minSlewRate * dt, delta));

  float clamped = m_last + delta;
  m_last = clamped;
  return clamped;
}

void RateLimiter::stepTo(float newValue) {
  m_last = newValue;
}

PIController::PIController(float kp, float ki, float intMax, float intMin) : m_kp(kp), m_ki(ki), m_intMax(intMax), m_intMin(intMin) {
}

float PIController::update(float deltaTime, float setpoint, float value) {
  float dv = setpoint - value;

  m_int += dv * deltaTime * m_ki;

  // Clamp to prevent windup
  m_int = min(m_intMax, max(m_intMin, m_int));

  return dv * m_kp + m_int;
}
