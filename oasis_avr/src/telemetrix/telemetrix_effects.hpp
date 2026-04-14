/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#pragma once

#include <stdint.h>

namespace OASIS
{
/*!\brief Telemetrix subsystem for MCU-managed effects */
class TelemetrixEffects
{
public:
  enum EffectKind : uint8_t
  {
    HELIPAD = 1,
  };

  enum EffectMode : uint8_t
  {
    DISABLED = 1,
    GUIDANCE = 2,
    LANDED = 3,
  };

  void ConfigureEffect(uint8_t effectKind,
                       uint8_t instanceId,
                       uint8_t analogPinCount,
                       uint8_t digitalPinCount,
                       uint8_t pwmPinCount,
                       const uint8_t* pinData);

  void SetEffect(uint8_t effectKind,
                 uint8_t instanceId,
                 uint8_t mode,
                 uint8_t valueCount,
                 const uint8_t* values);

  void Scan();
  void ResetData();

private:
  enum AnimationState : uint8_t
  {
    OFF = 0,
    GUIDANCE_ACTIVE = 1,
    LANDED_FADE = 2,
  };

  void UpdateGuidanceOutputs(uint32_t nowMs);
  void StartLandedFade(uint32_t nowMs);
  void UpdateLandedFade(uint32_t nowMs);
  void SetOutputs(float pairADutyCycle, float pairBDutyCycle);
  void SetOutputsOff();

  uint8_t m_irPin{0};
  uint8_t m_ledPairAPin{0};
  uint8_t m_ledPairBPin{0};
  uint8_t m_mode{DISABLED};
  uint8_t m_animationState{OFF};
  uint32_t m_guidanceStartedMs{0};
  uint32_t m_landedFadeStartedMs{0};
  float m_outputPairADutyCycle{0.0F};
  float m_outputPairBDutyCycle{0.0F};
  float m_landedFadeStartPairADutyCycle{0.0F};
  float m_landedFadeStartPairBDutyCycle{0.0F};
  bool m_attached{false};
};
} // namespace OASIS
