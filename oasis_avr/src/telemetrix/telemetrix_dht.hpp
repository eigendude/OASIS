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

#include <stdint.h>

// Max number of devices
#define MAX_DHTS 6

// DHT Report sub-types
#define DHT_DATA 0
#define DHT_READ_ERROR 1

class DHTStable;

namespace OASIS
{
class TelemetrixDHT
{
public:
  // Associate a pin with a DHT device
  void dht_new(uint8_t pin, uint8_t dhtType);

  void scan_dhts();

  void reset_data();

private:
  struct DHT
  {
    uint8_t pin;
    uint8_t dht_type;
    unsigned int last_value;
    DHTStable* dht_sensor;
  };

  // An array of DHT objects
  DHT dhts[MAX_DHTS];

  // Index into DHT struct
  uint8_t dht_index{0};

  // Scan DHT's every 2 seconds
  unsigned int dht_scan_interval{2000};

  // For analog input loop
  unsigned long dht_previous_millis{0};
};
} // namespace OASIS
