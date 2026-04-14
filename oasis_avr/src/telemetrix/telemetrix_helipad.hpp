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
/*!
 * \brief Telemetrix subsystem to control helipad guidance LEDs
 */
class TelemetrixHelipad
{
public:
  /*!
   * \brief High-level helipad modes driven by the host
   */
  enum Mode : uint8_t
  {
    DISABLED = 0,
    GUIDANCE = 1,
    LANDED = 2,
  };

  void Attach(uint8_t irPin, uint8_t ledPairAPin, uint8_t ledPairBPin);
  void Detach();
  void SetMode(uint8_t mode);

  void Scan();
  void ResetData();

private:
  enum AnimationState : uint8_t
  {
    OFF = 0,
    GUIDANCE_ACTIVE = 1,
    LANDED_FADE = 2,
  };

  enum GuidancePhase : uint8_t
  {
    PAIR_A_PULSE = 0,
    SHORT_GAP = 1,
    PAIR_B_PULSE = 2,
    LONG_GAP = 3,
  };

  void UpdateGuidanceOutputs(uint32_t nowMs);
  float GetGuidancePulseDutyCycle(uint32_t phaseElapsedMs, uint32_t pulseDurationMs) const;
  void StartLandedFade(uint32_t nowMs);
  void UpdateLandedFade(uint32_t nowMs);
  void SetOutputs(float pairADutyCycle, float pairBDutyCycle);
  void SetOutputsOff();

  uint8_t m_irPin{0};
  uint8_t m_ledPairAPin{0};
  uint8_t m_ledPairBPin{0};
  uint8_t m_mode{DISABLED};
  uint8_t m_animationState{OFF};
  uint8_t m_guidancePhase{PAIR_A_PULSE};
  uint32_t m_guidanceStartedMs{0};
  uint32_t m_landedFadeStartedMs{0};
  float m_outputPairADutyCycle{0.0F};
  float m_outputPairBDutyCycle{0.0F};
  float m_landedFadeStartPairADutyCycle{0.0F};
  float m_landedFadeStartPairBDutyCycle{0.0F};
  bool m_attached{false};
};
} // namespace OASIS
