/*
 *  Copyright (C) 2022 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */
#pragma once

#include "utils/timer.hpp"

#include <stdint.h>

#include <DHTStable.h>

namespace OASIS
{

using DHTScanCallback =
    void (*)(uint8_t pin, uint8_t dhtType, bool success, float humidity, float temperature);

class DHT
{
public:
  DHT(uint8_t pin, uint8_t dhtType);

  void Scan(DHTScanCallback scanCallback);

private:
  // Construction parameters
  const uint8_t m_pin;
  const uint8_t m_dhtType;

  // DHT parameters
  DHTStable m_sensor;

  // Timing parameters
  Timer m_timer;
};

} // namespace OASIS
