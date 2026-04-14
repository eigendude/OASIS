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
namespace EFFECTS
{
/*!
 * \brief Duty-cycle outputs for a small fixed-count PWM effect
 */
struct EffectOutputs
{
  // Number of valid entries in dutyCycles, expected range [0, 2]
  uint8_t outputCount{0};

  // Normalized duty cycles, where each entry is expected in [0.0, 1.0]
  float dutyCycles[2]{};
};

class EffectPrimitives
{
public:
  EffectPrimitives() = delete;

  static constexpr uint8_t kMaxOutputCount = 2;

  static float ClampDutyCycle(float value);
  static float ComputeSinePulse(uint32_t elapsedMs,
                                uint32_t periodMs,
                                float minDutyCycle,
                                float maxDutyCycle);
  static EffectOutputs ComputeAlternatingHalfSine(uint32_t elapsedMs, uint32_t periodMs);
  static EffectOutputs ScaleDutyCycles(const EffectOutputs& dutyCycles, float scale);
  static float MaxDutyCycle();
};
} // namespace EFFECTS
} // namespace OASIS
