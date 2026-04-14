/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "telemetrix_helipad.hpp"

#include <Arduino.h>

using namespace OASIS;

namespace
{
// Duration of the pair A guidance pulse in milliseconds
constexpr uint32_t PAIR_A_PULSE_DURATION_MS = 150;

// Gap between the A pulse and the B pulse in milliseconds
constexpr uint32_t SHORT_INTER_PULSE_GAP_MS = 70;

// Duration of the pair B guidance pulse in milliseconds
constexpr uint32_t PAIR_B_PULSE_DURATION_MS = 150;

// Gap before the pulse sequence repeats in milliseconds
constexpr uint32_t LONG_CYCLE_GAP_MS = 300;

// Ease-in and ease-out time at each pulse edge in milliseconds
constexpr uint32_t PULSE_EDGE_FADE_MS = 48;

// Fixed landed fade duration in milliseconds
constexpr uint32_t LANDED_FADE_DURATION_MS = 480;

// Normalized duty cycle used for the bright guidance pulses
constexpr float GUIDANCE_PULSE_DUTY_CYCLE = 0.92F;

constexpr float MAX_DUTY_CYCLE = 1.0F;
constexpr float PWM_MAX = 255.0F;

constexpr uint32_t GUIDANCE_CYCLE_DURATION_MS = PAIR_A_PULSE_DURATION_MS +
                                                SHORT_INTER_PULSE_GAP_MS +
                                                PAIR_B_PULSE_DURATION_MS + LONG_CYCLE_GAP_MS;

float EaseOutQuadratic(float progress)
{
  const float clampedProgress = constrain(progress, 0.0F, MAX_DUTY_CYCLE);
  const float inverseProgress = MAX_DUTY_CYCLE - clampedProgress;
  return MAX_DUTY_CYCLE - (inverseProgress * inverseProgress);
}

float EaseInOutQuadratic(float progress)
{
  const float clampedProgress = constrain(progress, 0.0F, MAX_DUTY_CYCLE);

  if (clampedProgress < 0.5F)
    return 2.0F * clampedProgress * clampedProgress;

  const float inverseProgress = MAX_DUTY_CYCLE - clampedProgress;
  return MAX_DUTY_CYCLE - (2.0F * inverseProgress * inverseProgress);
}
} // namespace

void TelemetrixHelipad::Attach(uint8_t irPin, uint8_t ledPairAPin, uint8_t ledPairBPin)
{
  m_irPin = irPin;
  m_ledPairAPin = ledPairAPin;
  m_ledPairBPin = ledPairBPin;
  m_guidancePhase = PAIR_A_PULSE;
  m_guidanceStartedMs = millis();
  m_landedFadeStartedMs = 0;
  m_mode = DISABLED;
  m_animationState = OFF;
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
  m_animationState = OFF;
  m_attached = false;
}

void TelemetrixHelipad::SetMode(uint8_t mode)
{
  if (!m_attached)
    return;

  const uint32_t nowMs = millis();

  if (mode == GUIDANCE)
  {
    m_guidancePhase = PAIR_A_PULSE;
    m_guidanceStartedMs = nowMs;
    m_mode = GUIDANCE;
    m_animationState = GUIDANCE_ACTIVE;
    return;
  }

  if (mode == LANDED)
  {
    m_mode = LANDED;

    if (m_animationState == GUIDANCE_ACTIVE)
      StartLandedFade(nowMs);
    else if (m_animationState != LANDED_FADE)
      SetOutputsOff();

    return;
  }

  m_mode = DISABLED;
  SetOutputsOff();
}

void TelemetrixHelipad::Scan()
{
  if (!m_attached)
    return;

  const uint32_t nowMs = millis();

  if (m_animationState == GUIDANCE_ACTIVE)
  {
    UpdateGuidanceOutputs(nowMs);
    return;
  }

  if (m_animationState == LANDED_FADE)
    UpdateLandedFade(nowMs);
}

void TelemetrixHelipad::ResetData()
{
  SetOutputsOff();
  m_mode = DISABLED;
  m_animationState = OFF;
  m_guidancePhase = PAIR_A_PULSE;
  m_guidanceStartedMs = 0;
  m_landedFadeStartedMs = 0;
  m_attached = false;
}

void TelemetrixHelipad::UpdateGuidanceOutputs(uint32_t nowMs)
{
  const uint32_t cycleElapsedMs = (nowMs - m_guidanceStartedMs) % GUIDANCE_CYCLE_DURATION_MS;

  float pairADutyCycle = 0.0F;
  float pairBDutyCycle = 0.0F;

  if (cycleElapsedMs < PAIR_A_PULSE_DURATION_MS)
  {
    m_guidancePhase = PAIR_A_PULSE;
    pairADutyCycle = GetGuidancePulseDutyCycle(cycleElapsedMs, PAIR_A_PULSE_DURATION_MS);
  }
  else if (cycleElapsedMs < PAIR_A_PULSE_DURATION_MS + SHORT_INTER_PULSE_GAP_MS)
  {
    m_guidancePhase = SHORT_GAP;
  }
  else if (cycleElapsedMs <
           PAIR_A_PULSE_DURATION_MS + SHORT_INTER_PULSE_GAP_MS + PAIR_B_PULSE_DURATION_MS)
  {
    m_guidancePhase = PAIR_B_PULSE;
    pairBDutyCycle = GetGuidancePulseDutyCycle(cycleElapsedMs - PAIR_A_PULSE_DURATION_MS -
                                                   SHORT_INTER_PULSE_GAP_MS,
                                               PAIR_B_PULSE_DURATION_MS);
  }
  else
  {
    m_guidancePhase = LONG_GAP;
  }

  SetOutputs(pairADutyCycle, pairBDutyCycle);
}

float TelemetrixHelipad::GetGuidancePulseDutyCycle(uint32_t phaseElapsedMs,
                                                   uint32_t pulseDurationMs) const
{
  if (PULSE_EDGE_FADE_MS == 0)
    return GUIDANCE_PULSE_DUTY_CYCLE;

  // Clamp the edge fade to half the pulse width so retuning cannot invert
  // the pulse into overlapping fades with no visible crest
  const uint32_t edgeFadeMs = min(PULSE_EDGE_FADE_MS, pulseDurationMs / 2);

  if (edgeFadeMs == 0)
    return GUIDANCE_PULSE_DUTY_CYCLE;

  float edgeScale = MAX_DUTY_CYCLE;

  if (phaseElapsedMs < edgeFadeMs)
  {
    const float fadeInProgress =
        static_cast<float>(phaseElapsedMs) / static_cast<float>(edgeFadeMs);
    edgeScale = EaseInOutQuadratic(fadeInProgress);
  }
  else
  {
    const uint32_t pulseRemainingMs = pulseDurationMs - phaseElapsedMs;

    if (pulseRemainingMs < edgeFadeMs)
    {
      const float fadeOutProgress =
          static_cast<float>(pulseRemainingMs) / static_cast<float>(edgeFadeMs);
      edgeScale = EaseInOutQuadratic(fadeOutProgress);
    }
  }

  return GUIDANCE_PULSE_DUTY_CYCLE * edgeScale;
}

void TelemetrixHelipad::StartLandedFade(uint32_t nowMs)
{
  UpdateGuidanceOutputs(nowMs);

  m_landedFadeStartedMs = nowMs;
  m_landedFadeStartPairADutyCycle = m_outputPairADutyCycle;
  m_landedFadeStartPairBDutyCycle = m_outputPairBDutyCycle;

  if (m_landedFadeStartPairADutyCycle <= 0.0F && m_landedFadeStartPairBDutyCycle <= 0.0F)
  {
    SetOutputsOff();
    return;
  }

  m_animationState = LANDED_FADE;
}

void TelemetrixHelipad::UpdateLandedFade(uint32_t nowMs)
{
  const uint32_t elapsedMs = nowMs - m_landedFadeStartedMs;

  if (elapsedMs >= LANDED_FADE_DURATION_MS)
  {
    SetOutputsOff();
    return;
  }

  // The landed animation uses a fixed-duration quadratic ease-out on the
  // fade amount so brightness falls smoothly and settles gently to black
  const float fadeProgress =
      static_cast<float>(elapsedMs) / static_cast<float>(LANDED_FADE_DURATION_MS);
  const float remainingScale = MAX_DUTY_CYCLE - EaseOutQuadratic(fadeProgress);

  SetOutputs(m_landedFadeStartPairADutyCycle * remainingScale,
             m_landedFadeStartPairBDutyCycle * remainingScale);
}

void TelemetrixHelipad::SetOutputs(float pairADutyCycle, float pairBDutyCycle)
{
  const float clampedPairA = constrain(pairADutyCycle, 0.0F, MAX_DUTY_CYCLE);
  const float clampedPairB = constrain(pairBDutyCycle, 0.0F, MAX_DUTY_CYCLE);

  m_outputPairADutyCycle = clampedPairA;
  m_outputPairBDutyCycle = clampedPairB;

  const int pairAPwm = static_cast<int>(clampedPairA * PWM_MAX);
  const int pairBPwm = static_cast<int>(clampedPairB * PWM_MAX);

  analogWrite(m_ledPairAPin, pairAPwm);
  analogWrite(m_ledPairBPin, pairBPwm);
}

void TelemetrixHelipad::SetOutputsOff()
{
  if (!m_attached)
    return;

  m_animationState = OFF;
  m_outputPairADutyCycle = 0.0F;
  m_outputPairBDutyCycle = 0.0F;

  analogWrite(m_ledPairAPin, 0);
  analogWrite(m_ledPairBPin, 0);
}
