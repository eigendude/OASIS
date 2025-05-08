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

#include "telemetrix_one_wire.hpp"

#include "telemetrix_commands.hpp"
#include "telemetrix_reports.hpp"

#include <HardwareSerial.h>
#include <OneWire.h>

using namespace OASIS;

void TelemetrixOneWire::onewire_init(uint8_t pin)
{
  m_oneWire = new OneWire(pin);
}

void TelemetrixOneWire::onewire_reset()
{
  const uint8_t reset_return = m_oneWire->reset();
  const uint8_t onewire_report_message[] = {3, ONE_WIRE_REPORT, ONE_WIRE_RESET, reset_return};

  Serial.write(onewire_report_message, 4);
}

void TelemetrixOneWire::onewire_select(uint8_t deviceAddress[8])
{
  m_oneWire->select(deviceAddress);
}

void TelemetrixOneWire::onewire_skip()
{
  m_oneWire->skip();
}

void TelemetrixOneWire::onewire_write(uint8_t value, bool power)
{
  // Write data and power values
  m_oneWire->write(value, power ? 1 : 0);
}

void TelemetrixOneWire::onewire_read()
{
  // onewire_report_message[0] = length of message including this element
  // onewire_report_message[1] = ONEWIRE_REPORT
  // onewire_report_message[2] = message subtype = 29
  // onewire_report_message[3] = data read

  const uint8_t data = m_oneWire->read();
  const uint8_t onewire_report_message[] = {3, ONE_WIRE_REPORT, ONE_WIRE_READ, data};

  Serial.write(onewire_report_message, 4);
}

void TelemetrixOneWire::onewire_reset_search()
{
  m_oneWire->reset_search();
}

void TelemetrixOneWire::onewire_search()
{
  uint8_t onewire_report_message[] = {
      10, ONE_WIRE_REPORT, ONE_WIRE_SEARCH, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

  m_oneWire->search(&onewire_report_message[3]);

  Serial.write(onewire_report_message, 11);
}

void TelemetrixOneWire::onewire_crc8(const uint8_t* address, uint8_t length)
{
  const uint8_t crc = m_oneWire->crc8(address, length);
  const uint8_t onewire_report_message[] = {3, ONE_WIRE_REPORT, ONE_WIRE_CRC8, crc};

  Serial.write(onewire_report_message, 4);
}
