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

#pragma once

#include <stdint.h>

// HC-SR04 Sonar Management
#define MAX_SONARS 6

class Ultrasonic;

namespace OASIS
{
class TelemetrixSonar
{
public:
  void sonar_new(uint8_t triggerPin, uint8_t echoPin);

  void scan_sonars();

  void reset_data();

private:
  struct Sonar
  {
    uint8_t trigger_pin;
    unsigned int last_value;
    Ultrasonic* usonic;
  };

  // An array of sonar objects
  Sonar sonars[MAX_SONARS];

  // Index into sonars struct
  uint8_t sonars_index{0};

  // Used for scanning the sonar devices.
  uint8_t last_sonar_visited{0};

  // Milliseconds between sensor pings (29ms is about the min)
  uint8_t sonar_scan_interval{33};

  // For analog input loop
  unsigned long sonar_previous_millis{0};
};
} // namespace OASIS
