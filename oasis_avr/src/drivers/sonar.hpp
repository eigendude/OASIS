/*
 *  Copyright (C) 2022-2023 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */
#pragma once

#include "utils/timer.hpp"

#include <stdint.h>

#include <Arduino.h>
#include <Ultrasonic.h>

class Ultrasonic;

namespace OASIS
{

using SonarScanCallback = void (*)(uint8_t digitalPin, unsigned int distance);

/*!
 * \brief HC-SR04 Sonar Management
 */
class Sonar
{
public:
  explicit Sonar(uint8_t triggerPin, uint8_t echoPin);

  uint8_t GetTriggerPin() const { return m_triggerPin; }
  uint8_t GetEchoPin() const { return m_echoPin; }

  void Scan(SonarScanCallback scanCallback);

private:
  // Construction parameters
  const uint8_t m_triggerPin;
  const uint8_t m_echoPin;

  // Sonar parameters
  Ultrasonic m_ultrasonic;
  unsigned int m_lastValue{0};

  // Timing parameters
  Timer m_scanTimer;
};

} // namespace OASIS
