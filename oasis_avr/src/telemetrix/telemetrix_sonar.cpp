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

#include "telemetrix_sonar.hpp"

#include "drivers/sonar.hpp"
#include "telemetrix_reports.hpp"

#include <string.h>

#include <Arduino.h>
#include <Ultrasonic.h>

using namespace OASIS;

void TelemetrixSonar::AttachSonar(uint8_t triggerPin, uint8_t echoPin)
{
  const int sonarIndex = GetSonarIndex();
  if (sonarIndex >= 0)
    m_sonars[sonarIndex] = new Sonar{triggerPin, echoPin};
}

void TelemetrixSonar::ScanSonars()
{
  for (unsigned int i = 0; i < MAX_SONARS; ++i)
  {
    if (m_sonars[i] == nullptr)
      continue;

    // Dereference iterator
    Sonar& sonar = *m_sonars[i];

    // Scan sonar
    sonar.Scan(
        [](uint8_t triggerPin, unsigned int distance)
        {
          // byte 0 = packet length
          // byte 1 = report type
          // byte 2 = trigger pin number
          // byte 3 = distance high order byte
          // byte 4 = distance low order byte
          const uint8_t reportMessage[5] = {4, SONAR_DISTANCE, triggerPin,
                                            static_cast<uint8_t>(distance >> 8),
                                            static_cast<uint8_t>(distance & 0xff)};

          Serial.write(reportMessage, 5);
        });
  }
}

void TelemetrixSonar::ResetData()
{
  for (unsigned int i = 0; i < MAX_SONARS; ++i)
  {
    if (m_sonars[i] != nullptr)
    {
      delete m_sonars[i];
      m_sonars[i] = nullptr;
    }
  }
}

int TelemetrixSonar::GetSonarIndex() const
{
  for (unsigned int i = 0; i < MAX_SONARS; ++i)
  {
    if (m_sonars[i] == nullptr)
      return i;
  }

  return -1;
}
