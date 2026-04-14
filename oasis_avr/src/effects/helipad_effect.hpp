/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#pragma once

#include "effects/effect_primitives.hpp"

#include <stdint.h>

namespace OASIS
{
namespace EFFECTS
{
/*!
 * \brief Helipad animation state machine independent from pin I/O
 */
class HelipadEffect
{
public:
  void Reset();

  void StartGuidance(uint32_t nowMs);

  // Returns true when outputs are updated immediately
  bool StartLanded(uint32_t nowMs);

  // Returns true when outputs are updated immediately
  bool Disable();

  // Returns true when outputs are updated this tick
  bool Tick(uint32_t nowMs);

  const EffectOutputs& GetOutputs() const { return m_outputs; }

private:
  enum AnimationState : uint8_t
  {
    OFF = 0,
    GUIDANCE_ACTIVE = 1,
    LANDED_FADE = 2,
  };

  static constexpr uint32_t kGuidancePeriodMs = 1200;

  // Fixed landed fade duration in milliseconds
  static constexpr uint32_t kLandedFadeDurationMs = 200;

  void UpdateGuidanceOutputs(uint32_t nowMs);
  void StartLandedFade(uint32_t nowMs);
  void UpdateLandedFade(uint32_t nowMs);
  bool SetOutputsOff();

  uint8_t m_animationState{OFF};
  uint32_t m_guidanceStartedMs{0};
  uint32_t m_landedFadeStartedMs{0};
  EffectOutputs m_outputs{2, {0.0F, 0.0F}};
  EffectOutputs m_landedFadeStartOutputs{2, {0.0F, 0.0F}};
};
} // namespace EFFECTS
} // namespace OASIS
