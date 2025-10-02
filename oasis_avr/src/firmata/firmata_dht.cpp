/*
 *  Copyright (C) 2021-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "firmata_dht.hpp"

#include "firmata_callbacks.hpp"

#include <Arduino.h>
#include <FirmataExpress.h>

using namespace OASIS;

namespace OASIS
{

// DHT constants
static constexpr uint8_t DHT_INTER_PING_INTERVAL_MS = 2200;
static constexpr uint8_t DHTLIB_OK = 0;

} // namespace OASIS

void FirmataDHT::Sample()
{
  if (m_numActiveDHTs != 0)
  {
    int rv;
    float humidity, temperature;

    const uint8_t current_pin = m_DHT_pinNumbers[m_nextDHT];
    const uint8_t current_type = m_DHT_type[m_nextDHT];

    m_dhtLoopCounter = 0;
    m_currentDHT = m_nextDHT;

    if (m_nextDHT++ >= m_numActiveDHTs - 1)
    {
      m_nextDHT = 0;
    }

    // Clear out the data buffer
    for (int i = 0; i < 4; i++)
      m_dhtValue[i] = static_cast<uint8_t>(0);

    if (current_type == 22)
      rv = m_DHT.read22(current_pin);
    else
      rv = m_DHT.read11(current_pin);

    if (rv == DHTLIB_OK)
    {
      float i = 0.0f;
      float f = 0.0f;

      humidity = m_DHT.getHumidity();

      f = modff(humidity, &i);

      m_dhtValue[0] = static_cast<uint8_t>(i);
      m_dhtValue[1] = static_cast<uint8_t>(f * 100);

      temperature = m_DHT.getTemperature();

      f = modff(temperature, &i);

      m_dhtValue[2] = static_cast<uint8_t>(i);
      m_dhtValue[3] = static_cast<uint8_t>(f * 100);
    }

    // Send the message back with an error status
    Firmata.write(START_SYSEX);
    Firmata.write(DHT_DATA);
    Firmata.write(current_pin);
    Firmata.write(current_type);
    Firmata.write(abs(rv));

    if (humidity >= 0.0)
      Firmata.write(0);
    else
      Firmata.write(1);

    if (temperature >= 0.0)
      Firmata.write(0);
    else
      Firmata.write(1);

    for (uint8_t i = 0; i < 4; ++i)
      Firmata.write(m_dhtValue[i]);

    Firmata.write(END_SYSEX);
  }
}

void FirmataDHT::EnableDHT(uint8_t digitalPin)
{
  Firmata.setPinMode(digitalPin, PIN_MODE_DHT);
}

void FirmataDHT::ConfigureDHT(int DHT_pin, int DHT_type)
{
  if (m_numActiveDHTs < MAX_DHTS)
  {
    if (DHT_type != 22 && DHT_type != 11)
    {
      Firmata.sendString("ERROR: UNKNOWN SENSOR TYPE, VALID SENSORS ARE 11, 22");
    }
    else
    {
      // Test the sensor
      m_DHT_pinNumbers[m_numActiveDHTs] = DHT_pin;
      m_DHT_type[m_numActiveDHTs] = DHT_type;

      FirmataCallbacks::SetPinModeCallback(DHT_pin, PIN_MODE_DHT);

      m_numActiveDHTs++;
      m_dhtNumLoops = m_dhtNumLoops / m_numActiveDHTs;
    }
  }
  else
  {
    Firmata.sendString("DHT_CONFIG Error: Exceeded number of supported DHT devices");
  }
}

void FirmataDHT::SetSamplingInterval(uint8_t samplingIntervalMs)
{
  // Calculate number of loops between each sample of DHT data
  m_dhtNumLoops = DHT_INTER_PING_INTERVAL_MS / samplingIntervalMs;
}
