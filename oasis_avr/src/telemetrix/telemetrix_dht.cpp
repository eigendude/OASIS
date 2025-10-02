/*
 *  Copyright (C) 2022-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  This file is derived from Telemetrix4Arduino under the AGPL 3.0 License
 *  Copyright (c) 2020-2021 Alan Yorinks
 *
 *  SPDX-License-Identifier: Apache-2.0 AND AGPL-3.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "telemetrix_dht.hpp"

#include "drivers/dht.hpp"
#include "telemetrix_reports.hpp"

#include <string.h>

#include <Arduino.h>
#include <HardwareSerial.h>

using namespace OASIS;

void TelemetrixDHT::NewDHT(uint8_t pin, uint8_t dhtType)
{
  const int dhtIndex = GetNextIndex();
  if (dhtIndex >= 0)
    m_dhts[dhtIndex] = new DHT{pin, dhtType};
}

void TelemetrixDHT::ScanDHTs()
{
  // Read and report all the dht sensors
  for (unsigned int i = 0; i < MAX_DHTS; ++i)
  {
    if (m_dhts[i] == nullptr)
      continue;

    m_dhts[i]->Scan(
        [](uint8_t pin, uint8_t dhtType, bool success, float humidity, float temperature)
        {
          // Data returned is in floating point form - 4 bytes
          // Each for humidity and temperature

          // byte 0 = packet length
          // byte 1 = report type
          // byte 2 = report sub type - DHT_DATA or DHT_ERROR
          // byte 3 = pin number
          // byte 4 = DHT type
          // byte 5 = humidity positivity flag 0=positive, 1= negative
          // byte 6 = temperature positivity flag 0=positive, 1= negative
          // byte 7 = humidity integer portion
          // byte 8 = humidity fractional portion
          // byte 9 = temperature integer portion
          // byte 10 = temperature fractional portion

          uint8_t report_message[11] = {
              10, DHT_REPORT, success ? DHT_DATA : DHT_READ_ERROR, pin, dhtType, 0, 0, 0, 0, 0, 0};

          // If success is false, this is an error report
          if (!success)
          {
            Serial.write(report_message, 11);
            return;
          }
          else
          {
            if (humidity >= 0.0)
              report_message[5] = 0;
            else
              report_message[5] = 1;

            float j;
            float f = modff(humidity, &j);
            report_message[7] = static_cast<uint8_t>(j);
            report_message[8] = static_cast<uint8_t>(f * 100);

            if (temperature >= 0.0)
              report_message[6] = 0;
            else
              report_message[6] = 1;

            f = modff(temperature, &j);
            report_message[9] = static_cast<uint8_t>(j);
            report_message[10] = static_cast<uint8_t>(f * 100);

            Serial.write(report_message, 11);
          }
        });
  }
}

void TelemetrixDHT::ResetData()
{
  for (unsigned int i = 0; i < MAX_DHTS; ++i)
  {
    if (m_dhts[i] != nullptr)
    {
      delete m_dhts[i];
      m_dhts[i] = nullptr;
    }
  }
}

int TelemetrixDHT::GetNextIndex() const
{
  for (unsigned int i = 0; i < MAX_DHTS; ++i)
  {
    if (m_dhts[i] == nullptr)
      return i;
  }

  return -1;
}
