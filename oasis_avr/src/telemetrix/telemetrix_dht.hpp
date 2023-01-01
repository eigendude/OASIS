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

// DHT Report sub-types
#define DHT_DATA 0
#define DHT_READ_ERROR 1

namespace OASIS
{
class DHT;

class TelemetrixDHT
{
public:
  // Associate a pin with a DHT device
  void NewDHT(uint8_t pin, uint8_t dhtType);

  void ScanDHTs();

  void ResetData();

private:
  // Max number of devices
  static constexpr unsigned int MAX_DHTS = 6;

  // Get the next unused index in the DHT array, or -1 if the array is full
  int GetNextIndex() const;

  // An array of DHT objects
  DHT* m_dhts[MAX_DHTS]{};
};
} // namespace OASIS
