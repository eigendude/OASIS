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
  void SetOutputs(float pairADutyCycle, float pairBDutyCycle);
  void SetOutputsOff();

  uint8_t m_irPin{0};
  uint8_t m_ledPairAPin{0};
  uint8_t m_ledPairBPin{0};
  uint8_t m_mode{DISABLED};
  uint32_t m_guidanceStartedMs{0};
  bool m_attached{false};
};
} // namespace OASIS
