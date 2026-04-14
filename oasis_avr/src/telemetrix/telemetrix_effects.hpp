/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#pragma once

#include "effects/helipad_effect.hpp"

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
    LED_THRUSTER = 2,
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
  static constexpr uint8_t kMaxEffectOutputs = 2;
  static constexpr uint8_t kHelipadAnalogPinCount = 1;
  static constexpr uint8_t kHelipadPwmPinCount = 2;
  static constexpr uint8_t kMaxHelipadInstances = 1;
  static constexpr uint8_t kMaxLedThrusterInstances = 1;

  struct HelipadInstance
  {
    uint8_t irPin{0};
    uint8_t pwmPins[kHelipadPwmPinCount]{};
    uint8_t mode{DISABLED};
    EFFECTS::HelipadEffect effect{};
    bool attached{false};
  };

  struct LedThrusterInstance
  {
    // Placeholder instance record for future LED thruster integration
    bool attached{false};
  };

  void ConfigureHelipad(uint8_t instanceId,
                        uint8_t analogPinCount,
                        uint8_t digitalPinCount,
                        uint8_t pwmPinCount,
                        const uint8_t* pinData);
  void SetHelipad(uint8_t instanceId, uint8_t mode, uint8_t valueCount, const uint8_t* values);
  void ScanHelipad(HelipadInstance& instance, uint32_t nowMs);
  void ResetHelipad(HelipadInstance& instance);
  void SetOutputs(const uint8_t* pwmPins,
                  uint8_t pwmPinCount,
                  const EFFECTS::EffectOutputs& outputs);
  void SetOutputsOff(const uint8_t* pwmPins, uint8_t pwmPinCount, bool attached);

  HelipadInstance m_helipadInstances[kMaxHelipadInstances]{};
  LedThrusterInstance m_ledThrusterInstances[kMaxLedThrusterInstances]{};
};
} // namespace OASIS
