/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "effects/led_thruster_effect.hpp"

using namespace OASIS;
using namespace OASIS::EFFECTS;

void LedThrusterEffect::Reset()
{
  m_enabled = false;
  m_runtimeMode = OFF;
  SetOutputsOff();
  m_idlePulseStartedMs = 0;
}

bool LedThrusterEffect::SetEnabled(bool enabled)
{
  if (!enabled)
  {
    m_enabled = false;
    m_runtimeMode = OFF;
    return SetOutputsOff();
  }

  const bool wasEnabled = m_enabled;
  m_enabled = true;

  if (!wasEnabled)
    m_idlePulseStartedMs = 0;

  return false;
}

bool LedThrusterEffect::SetMode(RuntimeMode mode)
{
  m_runtimeMode = mode;

  if (mode == OFF)
    return SetOutputsOff();

  if (mode == ACTIVE_FULL)
    return SetOutputsFull();

  m_idlePulseStartedMs = 0;

  return false;
}

bool LedThrusterEffect::Tick(uint32_t nowMs)
{
  const uint8_t previousState = m_animationState;
  UpdateStateFromMode();

  if (m_animationState == OFF)
    return SetOutputsOff();

  if (m_animationState == ACTIVE_FULL)
    return SetOutputsFull() || previousState != ACTIVE_FULL;

  if (m_idlePulseStartedMs == 0 || previousState != IDLE_PULSE)
    m_idlePulseStartedMs = nowMs;

  return UpdateIdlePulse(nowMs) || previousState != IDLE_PULSE;
}

void LedThrusterEffect::UpdateStateFromMode()
{
  if (!m_enabled)
  {
    m_animationState = OFF;
    return;
  }

  m_animationState = m_runtimeMode;
}

bool LedThrusterEffect::SetOutputsOff()
{
  const bool changed = m_outputs.dutyCycles[0] != 0.0F || m_animationState != OFF;

  m_animationState = OFF;
  m_outputs.outputCount = 1;
  m_outputs.dutyCycles[0] = 0.0F;
  m_outputs.dutyCycles[1] = 0.0F;

  return changed;
}

bool LedThrusterEffect::SetOutputsFull()
{
  const float fullDutyCycle = EffectPrimitives::MaxDutyCycle();
  const bool changed = m_outputs.dutyCycles[0] != fullDutyCycle;

  m_outputs.outputCount = 1;
  m_outputs.dutyCycles[0] = fullDutyCycle;
  m_outputs.dutyCycles[1] = 0.0F;

  return changed;
}

bool LedThrusterEffect::UpdateIdlePulse(uint32_t nowMs)
{
  const uint32_t elapsedMs = nowMs - m_idlePulseStartedMs;
  const float pulseDutyCycle = EffectPrimitives::ComputeSinePulse(
      elapsedMs, kIdlePulsePeriodMs, kIdleMinDutyCycle, kIdleMaxDutyCycle);

  const bool changed = m_outputs.dutyCycles[0] != pulseDutyCycle;

  m_outputs.outputCount = 1;
  m_outputs.dutyCycles[0] = pulseDutyCycle;
  m_outputs.dutyCycles[1] = 0.0F;

  return changed;
}
