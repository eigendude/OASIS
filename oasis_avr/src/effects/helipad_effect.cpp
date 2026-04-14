/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "effects/helipad_effect.hpp"

using namespace OASIS;
using namespace OASIS::EFFECTS;

void HelipadEffect::Reset()
{
  SetOutputsOff();
  m_animationState = OFF;
  m_guidanceStartedMs = 0;
  m_landedFadeStartedMs = 0;
}

void HelipadEffect::StartGuidance(uint32_t nowMs)
{
  m_guidanceStartedMs = nowMs;
  m_animationState = GUIDANCE_ACTIVE;
}

bool HelipadEffect::StartLanded(uint32_t nowMs)
{
  if (m_animationState == GUIDANCE_ACTIVE)
  {
    StartLandedFade(nowMs);
    return true;
  }

  if (m_animationState != LANDED_FADE)
    return SetOutputsOff();

  return false;
}

bool HelipadEffect::Disable()
{
  return SetOutputsOff();
}

bool HelipadEffect::Tick(uint32_t nowMs)
{
  if (m_animationState == GUIDANCE_ACTIVE)
  {
    UpdateGuidanceOutputs(nowMs);
    return true;
  }

  if (m_animationState == LANDED_FADE)
  {
    UpdateLandedFade(nowMs);
    return true;
  }

  return false;
}

void HelipadEffect::UpdateGuidanceOutputs(uint32_t nowMs)
{
  const uint32_t elapsedMs = nowMs - m_guidanceStartedMs;

  // One full beacon cycle spans both LED pairs. Each half-cycle drives one
  // pair with a sine pulse matching the previous host-side animation.
  m_outputs = EffectPrimitives::ComputeAlternatingHalfSine(elapsedMs, kGuidancePeriodMs);
}

void HelipadEffect::StartLandedFade(uint32_t nowMs)
{
  UpdateGuidanceOutputs(nowMs);

  m_landedFadeStartedMs = nowMs;
  m_landedFadeStartOutputs = m_outputs;

  if (m_landedFadeStartOutputs.dutyCycles[0] <= 0.0F &&
      m_landedFadeStartOutputs.dutyCycles[1] <= 0.0F)
  {
    SetOutputsOff();
    return;
  }

  m_animationState = LANDED_FADE;
}

void HelipadEffect::UpdateLandedFade(uint32_t nowMs)
{
  const uint32_t elapsedMs = nowMs - m_landedFadeStartedMs;

  if (elapsedMs >= kLandedFadeDurationMs)
  {
    SetOutputsOff();
    return;
  }

  // The landed animation uses a fixed-duration linear fade so the MCU owns
  // the full transition without depending on host-side frame timing.
  const float fadeProgress =
      static_cast<float>(elapsedMs) / static_cast<float>(kLandedFadeDurationMs);
  const float remainingScale = EffectPrimitives::MaxDutyCycle() - fadeProgress;

  m_outputs = EffectPrimitives::ScaleDutyCycles(m_landedFadeStartOutputs, remainingScale);
}

bool HelipadEffect::SetOutputsOff()
{
  const bool outputChanged =
      m_animationState != OFF || m_outputs.dutyCycles[0] > 0.0F || m_outputs.dutyCycles[1] > 0.0F;

  m_animationState = OFF;
  m_outputs.outputCount = 2;
  m_outputs.dutyCycles[0] = 0.0F;
  m_outputs.dutyCycles[1] = 0.0F;

  return outputChanged;
}
