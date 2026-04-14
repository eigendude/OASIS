/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "telemetrix_helipad.hpp"

#include <math.h>

#include <Arduino.h>

using namespace OASIS;

namespace
{
constexpr uint32_t GUIDANCE_PERIOD_MS = 1200;

constexpr float HALF_CYCLE = 0.5F;
constexpr float MAX_DUTY_CYCLE = 1.0F;
constexpr float PI_F = 3.14159265358979323846F;
constexpr float PWM_MAX = 255.0F;
} // namespace

void TelemetrixHelipad::Attach(uint8_t irPin, uint8_t ledPairAPin, uint8_t ledPairBPin)
{
  m_irPin = irPin;
  m_ledPairAPin = ledPairAPin;
  m_ledPairBPin = ledPairBPin;
  m_guidanceStartedMs = millis();
  m_mode = DISABLED;
  m_attached = true;

  pinMode(m_ledPairAPin, OUTPUT);
  pinMode(m_ledPairBPin, OUTPUT);
  SetOutputsOff();
}

void TelemetrixHelipad::Detach()
{
  if (!m_attached)
    return;

  SetOutputsOff();
  m_mode = DISABLED;
  m_attached = false;
}

void TelemetrixHelipad::SetMode(uint8_t mode)
{
  if (!m_attached)
    return;

  if (mode == GUIDANCE && m_mode != GUIDANCE)
    m_guidanceStartedMs = millis();

  m_mode = mode;

  if (m_mode != GUIDANCE)
    SetOutputsOff();
}

void TelemetrixHelipad::Scan()
{
  if (!m_attached || m_mode != GUIDANCE)
    return;

  const uint32_t elapsedMs = millis() - m_guidanceStartedMs;

  // One full beacon cycle spans both LED pairs. Each half-cycle drives one
  // pair with a sine pulse matching the previous host-side animation.
  const float cyclePhase =
      static_cast<float>(elapsedMs % GUIDANCE_PERIOD_MS) / static_cast<float>(GUIDANCE_PERIOD_MS);

  float pairADutyCycle = 0.0F;
  float pairBDutyCycle = 0.0F;

  if (cyclePhase < HALF_CYCLE)
  {
    const float localPhase = cyclePhase / HALF_CYCLE;
    pairADutyCycle = sinf(PI_F * localPhase) * MAX_DUTY_CYCLE;
  }
  else
  {
    const float localPhase = (cyclePhase - HALF_CYCLE) / HALF_CYCLE;
    pairBDutyCycle = sinf(PI_F * localPhase) * MAX_DUTY_CYCLE;
  }

  SetOutputs(pairADutyCycle, pairBDutyCycle);
}

void TelemetrixHelipad::ResetData()
{
  SetOutputsOff();
  m_mode = DISABLED;
  m_guidanceStartedMs = 0;
  m_attached = false;
}

void TelemetrixHelipad::SetOutputs(float pairADutyCycle, float pairBDutyCycle)
{
  const float clampedPairA = constrain(pairADutyCycle, 0.0F, MAX_DUTY_CYCLE);
  const float clampedPairB = constrain(pairBDutyCycle, 0.0F, MAX_DUTY_CYCLE);

  const int pairAPwm = static_cast<int>(clampedPairA * PWM_MAX);
  const int pairBPwm = static_cast<int>(clampedPairB * PWM_MAX);

  analogWrite(m_ledPairAPin, pairAPwm);
  analogWrite(m_ledPairBPin, pairBPwm);
}

void TelemetrixHelipad::SetOutputsOff()
{
  if (!m_attached)
    return;

  analogWrite(m_ledPairAPin, 0);
  analogWrite(m_ledPairBPin, 0);
}
