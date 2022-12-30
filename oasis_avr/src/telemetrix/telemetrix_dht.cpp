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

#include "telemetrix_dht.hpp"

#include "telemetrix_reports.hpp"

#include <string.h>

#include <DHTStable.h>
#include <HardwareSerial.h>

using namespace OASIS;

void TelemetrixDHT::dht_new(uint8_t pin, uint8_t dhtType)
{
  if (dht_index < MAX_DHTS)
  {
    dhts[dht_index].dht_sensor = new DHTStable();

    dhts[dht_index].pin = pin;
    dhts[dht_index].dht_type = dhtType;
    dht_index++;
  }
}

void TelemetrixDHT::scan_dhts()
{
  // Prebuild report for valid data
  // Reuse the report if a read command fails

  // Data returned is in floating point form - 4 bytes
  // Each for humidity and temperature

  // byte 0 = packet length
  // byte 1 = report type
  // byte 2 = report sub type - DHT_DATA or DHT_ERROR
  // byte 3 = pin number
  // byte 4 = dht type
  // byte 5 = humidity positivity flag 0=positive, 1= negative
  // byte 6 = temperature positivity flag 0=positive, 1= negative
  // byte 7 = humidity integer portion
  // byte 8 = humidity fractional portion
  // byte 9 = temperature integer portion
  // byte 10= temperature fractional portion

  uint8_t report_message[11] = {10, DHT_REPORT, DHT_DATA, 0, 0, 0, 0, 0, 0, 0, 0};

  // Are there any dhts to read?
  if (dht_index)
  {
    // Is it time to do the read? This should occur every 2 seconds
    const unsigned long dht_current_millis = millis();
    if (dht_current_millis - dht_previous_millis > dht_scan_interval)
    {
      // Update for the next scan
      dht_previous_millis = dht_current_millis;

      // Read and report all the dht sensors
      for (unsigned int i = 0; i < dht_index; ++i)
      {
        // Error type in report_message[2] will be set further down
        report_message[3] = dhts[i].pin;
        report_message[4] = dhts[i].dht_type;

        // Read the device
        const int retVal = dhts[i].dht_type == 22 ? dhts[i].dht_sensor->read22(dhts[i].pin)
                                                  : dhts[i].dht_sensor->read11(dhts[i].pin);
        report_message[2] = static_cast<uint8_t>(retVal);

        // If retVal is not zero, this is an error report
        if (retVal != 0)
        {
          Serial.write(report_message, 11);
          return;
        }
        else
        {
          float humidity = dhts[i].dht_sensor->getHumidity();
          if (humidity >= 0.0)
            report_message[5] = 0;
          else
            report_message[5] = 1;

          float j;
          float f = modff(humidity, &j);
          report_message[7] = static_cast<uint8_t>(j);
          report_message[8] = static_cast<uint8_t>(f * 100);

          float temperature = dhts[i].dht_sensor->getTemperature();
          if (temperature >= 0.0)
            report_message[6] = 0;
          else
            report_message[6] = 1;

          f = modff(temperature, &j);
          report_message[9] = static_cast<uint8_t>(j);
          report_message[10] = static_cast<uint8_t>(f * 100);
          Serial.write(report_message, 11);
        }
      }
    }
  }
}

void TelemetrixDHT::reset_data()
{
  memset(dhts, 0, sizeof(dhts));

  dht_index = 0;
  dht_scan_interval = 2000;
  dht_previous_millis = 0;
}
