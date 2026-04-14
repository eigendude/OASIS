/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "effects/effect_primitives.hpp"

#include <math.h>

using namespace OASIS;
using namespace OASIS::EFFECTS;

namespace
{
constexpr float kMaxDutyCycle = 1.0F;
constexpr float kHalfCycle = 0.5F;
constexpr float kPi = 3.14159265358979323846F;
} // namespace

float EffectPrimitives::ClampDutyCycle(float value)
{
  if (value < 0.0F)
    return 0.0F;

  if (value > kMaxDutyCycle)
    return kMaxDutyCycle;

  return value;
}

EffectOutputs EffectPrimitives::ComputeAlternatingHalfSine(uint32_t elapsedMs, uint32_t periodMs)
{
  EffectOutputs output;
  output.outputCount = 2;

  if (periodMs == 0)
    return output;

  const float cyclePhase = static_cast<float>(elapsedMs % periodMs) / static_cast<float>(periodMs);

  if (cyclePhase < kHalfCycle)
  {
    const float localPhase = cyclePhase / kHalfCycle;
    output.dutyCycles[0] = sinf(kPi * localPhase) * kMaxDutyCycle;
  }
  else
  {
    const float localPhase = (cyclePhase - kHalfCycle) / kHalfCycle;
    output.dutyCycles[1] = sinf(kPi * localPhase) * kMaxDutyCycle;
  }

  return output;
}

EffectOutputs EffectPrimitives::ScaleDutyCycles(const EffectOutputs& dutyCycles, float scale)
{
  EffectOutputs output;
  output.outputCount = dutyCycles.outputCount;

  if (output.outputCount > 0)
    output.dutyCycles[0] = ClampDutyCycle(dutyCycles.dutyCycles[0] * scale);

  if (output.outputCount > 1)
    output.dutyCycles[1] = ClampDutyCycle(dutyCycles.dutyCycles[1] * scale);

  return output;
}

float EffectPrimitives::MaxDutyCycle()
{
  return kMaxDutyCycle;
}
