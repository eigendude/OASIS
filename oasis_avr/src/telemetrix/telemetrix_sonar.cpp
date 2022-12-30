/*
 *  Copyright (C) 2022 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  This file is derived from Telemetrix4Arduino under the AGPL 3.0 License
 *  Copyright (c) 2020-2021 Alan Yorinks
 *
 *  SPDX-License-Identifier: Apache-2.0 AND AGPL-3.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "telemetrix_sonar.hpp"

#include "telemetrix_reports.hpp"

#include <string.h>

#include <Arduino.h>
#include <Ultrasonic.h>

using namespace OASIS;

void TelemetrixSonar::sonar_new(uint8_t triggerPin, uint8_t echoPin)
{
  sonars[sonars_index].usonic = new Ultrasonic(triggerPin, echoPin, 80000UL);
  sonars[sonars_index].trigger_pin = triggerPin;
  sonars_index++;
}

void TelemetrixSonar::scan_sonars()
{
  if (sonars_index)
  {
    const unsigned long sonar_current_millis = millis();
    if (sonar_current_millis - sonar_previous_millis > sonar_scan_interval)
    {
      sonar_previous_millis += sonar_scan_interval;

      const unsigned int distance = sonars[last_sonar_visited].usonic->read();
      if (distance != sonars[last_sonar_visited].last_value)
      {
        sonars[last_sonar_visited].last_value = distance;

        // byte 0 = packet length
        // byte 1 = report type
        // byte 2 = trigger pin number
        // byte 3 = distance high order byte
        // byte 4 = distance low order byte
        const uint8_t reportMessage[5] = {4, SONAR_DISTANCE, sonars[last_sonar_visited].trigger_pin,
                                          static_cast<uint8_t>(distance >> 8),
                                          static_cast<uint8_t>(distance & 0xff)};

        Serial.write(reportMessage, 5);
      }

      last_sonar_visited++;
      if (last_sonar_visited == sonars_index)
        last_sonar_visited = 0;
    }
  }
}

void TelemetrixSonar::reset_data()
{
  memset(sonars, 0, sizeof(sonars));

  sonars_index = 0;
  sonar_scan_interval = 33;
  sonar_previous_millis = 0;
}
