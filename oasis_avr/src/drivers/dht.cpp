/*
 *  Copyright (C) 2022 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "dht.hpp"

using namespace OASIS;

namespace
{
// Scan DHT's every 2 seconds
constexpr unsigned int DHT_SCAN_INTERVAL = 2000;
} // namespace

DHT::DHT(uint8_t pin, uint8_t dhtType) : m_pin(pin), m_dhtType(dhtType)
{
}

void DHT::Scan(DHTScanCallback scanCallback)
{
  if (!m_timer.IsExpired())
  {
    // Timeout not elapsed yet
    return;
  }

  m_timer.SetTimeout(DHT_SCAN_INTERVAL);

  // Read the device
  const int retVal = m_dhtType == 22 ? m_sensor.read22(m_pin) : m_sensor.read11(m_pin);

  // Get measurements if read was successful
  const bool success = (retVal == 0);
  const float humidity = success ? m_sensor.getHumidity() : 0.0f;
  const float temperature = success ? m_sensor.getTemperature() : 0.0f;

  // Report emasurements
  scanCallback(m_pin, m_dhtType, success, humidity, temperature);
}
