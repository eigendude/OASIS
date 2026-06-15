/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "telemetrix_effects.hpp"

#include <Arduino.h>

using namespace OASIS;
using namespace OASIS::EFFECTS;

namespace
{
// LED_THRUSTER mode values mirrored from oasis_msgs/msg/EffectMode.msg
constexpr uint8_t kLedThrusterDisabledMode = 10;
constexpr uint8_t kLedThrusterIdleMode = 11;
constexpr uint8_t kLedThrusterMovingMode = 12;

} // namespace

void TelemetrixEffects::ConfigureEffect(uint8_t effectKind,
                                        uint8_t instanceId,
                                        uint8_t analogPinCount,
                                        uint8_t digitalPinCount,
                                        uint8_t pwmPinCount,
                                        const uint8_t* pinData)
{
  switch (effectKind)
  {
    case HELIPAD:
      ConfigureHelipad(instanceId, analogPinCount, digitalPinCount, pwmPinCount, pinData);
      return;
    case LED_THRUSTER:
      ConfigureLedThruster(instanceId, analogPinCount, digitalPinCount, pwmPinCount, pinData);
      return;
    default:
      return;
  }
}

void TelemetrixEffects::SetEffect(
    uint8_t effectKind, uint8_t instanceId, uint8_t mode, uint8_t valueCount, const uint8_t* values)
{
  switch (effectKind)
  {
    case HELIPAD:
      SetHelipad(instanceId, mode, valueCount, values);
      return;
    case LED_THRUSTER:
      SetLedThruster(instanceId, mode);
      return;
    default:
      return;
  }
}

void TelemetrixEffects::Scan()
{
  const uint32_t nowMs = millis();

  for (uint8_t i = 0; i < kMaxHelipadInstances; ++i)
    ScanHelipad(m_helipadInstances[i], nowMs);

  for (uint8_t i = 0; i < kMaxLedThrusterInstances; ++i)
    ScanLedThruster(m_ledThrusterInstances[i], nowMs);
}

void TelemetrixEffects::ResetData()
{
  for (uint8_t i = 0; i < kMaxHelipadInstances; ++i)
    ResetHelipad(m_helipadInstances[i]);

  for (uint8_t i = 0; i < kMaxLedThrusterInstances; ++i)
    ResetLedThruster(m_ledThrusterInstances[i]);
}

void TelemetrixEffects::ConfigureHelipad(uint8_t instanceId,
                                         uint8_t analogPinCount,
                                         uint8_t digitalPinCount,
                                         uint8_t pwmPinCount,
                                         const uint8_t* pinData)
{
  if (instanceId >= kMaxHelipadInstances)
    return;

  if (analogPinCount < kHelipadAnalogPinCount || pwmPinCount < kHelipadPwmPinCount)
    return;

  if (pinData == nullptr)
    return;

  HelipadInstance& instance = m_helipadInstances[instanceId];

  const uint8_t analogPinOffset = 0;
  const uint8_t pwmPinOffset = static_cast<uint8_t>(analogPinCount + digitalPinCount);

  instance.irPin = pinData[analogPinOffset];
  instance.pwmPins[0] = pinData[pwmPinOffset];
  instance.pwmPins[1] = pinData[pwmPinOffset + 1];
  instance.effect.Reset();
  instance.mode = DISABLED;
  instance.attached = true;

  pinMode(instance.pwmPins[0], OUTPUT);
  pinMode(instance.pwmPins[1], OUTPUT);
  SetOutputsOff(instance.pwmPins, kHelipadPwmPinCount, instance.attached);
}

void TelemetrixEffects::SetHelipad(uint8_t instanceId,
                                   uint8_t mode,
                                   uint8_t valueCount,
                                   const uint8_t* values)
{
  if (instanceId >= kMaxHelipadInstances)
    return;

  HelipadInstance& instance = m_helipadInstances[instanceId];

  if (!instance.attached)
    return;

  if (valueCount > 0 && values == nullptr)
    return;

  const uint32_t nowMs = millis();

  if (mode == GUIDANCE)
  {
    instance.effect.StartGuidance(nowMs);
    instance.mode = GUIDANCE;
    return;
  }

  if (mode == LANDED)
  {
    instance.mode = LANDED;

    if (instance.effect.StartLanded(nowMs))
      SetOutputs(instance.pwmPins, kHelipadPwmPinCount, instance.effect.GetOutputs());

    return;
  }

  instance.mode = DISABLED;
  instance.effect.Disable();
  SetOutputsOff(instance.pwmPins, kHelipadPwmPinCount, instance.attached);
}

void TelemetrixEffects::ConfigureLedThruster(uint8_t instanceId,
                                             uint8_t analogPinCount,
                                             uint8_t digitalPinCount,
                                             uint8_t pwmPinCount,
                                             const uint8_t* pinData)
{
  if (instanceId >= kMaxLedThrusterInstances)
    return;

  if (analogPinCount != 0 || digitalPinCount != 0 || pwmPinCount < kLedThrusterPwmPinCount)
  {
    return;
  }

  if (pinData == nullptr)
    return;

  LedThrusterInstance& instance = m_ledThrusterInstances[instanceId];

  const uint8_t pwmPinOffset = 0;

  instance.pwmPin = pinData[pwmPinOffset];
  instance.mode = kLedThrusterDisabledMode;
  instance.effect.Reset();
  instance.attached = true;

  pinMode(instance.pwmPin, OUTPUT);
  SetOutput(instance.pwmPin, instance.effect.GetOutputs());
}

void TelemetrixEffects::SetLedThruster(uint8_t instanceId, uint8_t mode)
{
  if (instanceId >= kMaxLedThrusterInstances)
    return;

  LedThrusterInstance& instance = m_ledThrusterInstances[instanceId];

  if (!instance.attached)
    return;

  instance.mode = mode;

  if (mode == kLedThrusterDisabledMode)
  {
    if (instance.effect.SetEnabled(false))
      SetOutput(instance.pwmPin, instance.effect.GetOutputs());

    return;
  }

  bool outputsChanged = instance.effect.SetEnabled(true);

  if (mode == kLedThrusterMovingMode)
  {
    outputsChanged = instance.effect.SetMode(LedThrusterEffect::ACTIVE_FULL) || outputsChanged;
  }
  else if (mode == kLedThrusterIdleMode)
  {
    outputsChanged = instance.effect.SetMode(LedThrusterEffect::IDLE_PULSE) || outputsChanged;
  }
  else
  {
    outputsChanged = instance.effect.SetMode(LedThrusterEffect::OFF) || outputsChanged;
  }

  if (outputsChanged)
    SetOutput(instance.pwmPin, instance.effect.GetOutputs());
}

void TelemetrixEffects::ScanHelipad(HelipadInstance& instance, uint32_t nowMs)
{
  if (!instance.attached)
    return;

  if (instance.effect.Tick(nowMs))
    SetOutputs(instance.pwmPins, kHelipadPwmPinCount, instance.effect.GetOutputs());
}

void TelemetrixEffects::ResetHelipad(HelipadInstance& instance)
{
  SetOutputsOff(instance.pwmPins, kHelipadPwmPinCount, instance.attached);
  instance.mode = DISABLED;
  instance.effect.Reset();
  instance.attached = false;
}

void TelemetrixEffects::ScanLedThruster(LedThrusterInstance& instance, uint32_t nowMs)
{
  if (!instance.attached)
    return;

  if (instance.effect.Tick(nowMs))
    SetOutput(instance.pwmPin, instance.effect.GetOutputs());
}

void TelemetrixEffects::ResetLedThruster(LedThrusterInstance& instance)
{
  if (instance.attached)
    analogWrite(instance.pwmPin, 0);

  instance.mode = kLedThrusterDisabledMode;
  instance.effect.Reset();
  instance.attached = false;
}

void TelemetrixEffects::SetOutputs(const uint8_t* pwmPins,
                                   uint8_t pwmPinCount,
                                   const EffectOutputs& outputs)
{
  if (pwmPins == nullptr)
    return;

  const float pwmMax = 255.0F;
  const uint8_t outputCount =
      outputs.outputCount > kMaxEffectOutputs ? kMaxEffectOutputs : outputs.outputCount;

  for (uint8_t i = 0; i < pwmPinCount; ++i)
  {
    const float dutyCycle =
        i < outputCount ? EffectPrimitives::ClampDutyCycle(outputs.dutyCycles[i]) : 0.0F;
    const int pwm = static_cast<int>(dutyCycle * pwmMax);

    analogWrite(pwmPins[i], pwm);
  }
}

void TelemetrixEffects::SetOutput(uint8_t pwmPin, const EffectOutputs& outputs)
{
  SetOutputs(&pwmPin, 1, outputs);
}

void TelemetrixEffects::SetOutputsOff(const uint8_t* pwmPins, uint8_t pwmPinCount, bool attached)
{
  if (!attached || pwmPins == nullptr)
    return;

  for (uint8_t i = 0; i < pwmPinCount; ++i)
    analogWrite(pwmPins[i], 0);
}
