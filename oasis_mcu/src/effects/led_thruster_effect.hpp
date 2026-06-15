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
 * \brief Falcon LED thruster animation state machine independent from pin I/O
 */
class LedThrusterEffect
{
public:
  enum RuntimeMode : uint8_t
  {
    OFF = 0,
    IDLE_PULSE = 1,
    ACTIVE_FULL = 2,
  };

  void Reset();

  // Enable or disable the thruster animation output
  bool SetEnabled(bool enabled);

  // Select explicit runtime behavior for the thruster LED animation
  bool SetMode(RuntimeMode mode);

  // Returns true when outputs are updated this tick
  bool Tick(uint32_t nowMs);

  const EffectOutputs& GetOutputs() const { return m_outputs; }

private:
  // Idle pulse period in milliseconds for stationary thruster glow
  static constexpr uint32_t kIdlePulsePeriodMs = 1800;

  // Lower idle pulse brightness bound, normalized duty cycle [0.0, 1.0]
  static constexpr float kIdleMinDutyCycle = 0.10F;

  // Upper idle pulse brightness bound, normalized duty cycle [0.0, 1.0]
  static constexpr float kIdleMaxDutyCycle = 1.00F;

  void UpdateStateFromMode();
  bool SetOutputsOff();
  bool SetOutputsFull();
  bool UpdateIdlePulse(uint32_t nowMs);

  bool m_enabled{false};
  RuntimeMode m_runtimeMode{OFF};
  uint8_t m_animationState{OFF};
  uint32_t m_idlePulseStartedMs{0};
  EffectOutputs m_outputs{1, {0.0F, 0.0F}};
};
} // namespace EFFECTS
} // namespace OASIS
