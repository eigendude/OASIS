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

class OneWire;

namespace OASIS
{
class TelemetrixOneWire
{
public:
  void onewire_init(uint8_t pin);

  void onewire_reset();
  void onewire_select(uint8_t deviceAddress[8]);
  void onewire_skip();
  void onewire_write(uint8_t value, bool power);
  void onewire_read();
  void onewire_reset_search();
  void onewire_search();
  void onewire_crc8(const uint8_t* address, uint8_t length);

private:
  // A pointer to a OneWire object
  OneWire* m_oneWire{nullptr};
};
} // namespace OASIS
