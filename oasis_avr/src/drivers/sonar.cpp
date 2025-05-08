/*
 *  Copyright (C) 2022-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "sonar.hpp"

using namespace OASIS;

namespace
{
// Sonar read timeout
constexpr uint32_t SONAR_TIMEOUT = 80000UL;

// Milliseconds between sensor pings (29ms is about the min)
constexpr uint8_t SONAR_SCAN_INTERVAL = 33;
} // namespace

Sonar::Sonar(uint8_t triggerPin, uint8_t echoPin)
  : m_triggerPin(triggerPin), m_ultrasonic(triggerPin, echoPin, SONAR_TIMEOUT)
{
}

void Sonar::Scan(SonarScanCallback scanCallback)
{
  if (!m_scanTimer.IsExpired())
  {
    // Timer hasn't elapsed yet
    return;
  }

  m_scanTimer.SetTimeout(SONAR_SCAN_INTERVAL);

  const unsigned int distance = m_ultrasonic.read();
  if (distance != m_lastValue)
  {
    m_lastValue = distance;
    scanCallback(m_triggerPin, distance);
  }
}
