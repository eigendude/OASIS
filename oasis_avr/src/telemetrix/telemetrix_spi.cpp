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

#include "telemetrix_spi.hpp"

#include "telemetrix_reports.hpp"

#include <HardwareSerial.h>
#include <SPI.h>

using namespace OASIS;

void TelemetrixSPI::init_spi(uint8_t pinCount, const uint8_t* pins)
{
  // Initialize chip select GPIO pins
  for (unsigned int i = 0; i < pinCount; ++i)
  {
    const uint8_t csPin = pins[i];

    // Chip select is active-low, so we'll initialize it to a driven-high state
    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);
  }

  SPI.begin();
}

void TelemetrixSPI::write_blocking_spi(uint8_t byteCount, const uint8_t* data)
{
  for (unsigned int i = 0; i < byteCount; ++i)
    SPI.transfer(data[i]);
}

void TelemetrixSPI::read_blocking_spi(uint8_t byteCount, uint8_t readRegister)
{
  // spi_report_message[0] = length of message including this element
  // spi_report_message[1] = SPI_REPORT
  // spi_report_message[2] = register used for the read
  // spi_report_message[3] = number of bytes returned
  // spi_report_message[4..] = data read

  // Configure the report message
  // Calculate the packet length
  spi_report_message[0] = byteCount + 3; // Packet length
  spi_report_message[1] = SPI_REPORT;
  spi_report_message[2] = readRegister; // Register
  spi_report_message[3] = byteCount; // Number of bytes read

  // Write the register out. OR it with 0x80 to indicate a read
  SPI.transfer(readRegister | 0x80);

  // Now read the specified number of bytes and place them in the report buffer
  for (unsigned int i = 0; i < byteCount; ++i)
    spi_report_message[i + 4] = SPI.transfer(0x00);

  Serial.write(spi_report_message, byteCount + 4);
}

void TelemetrixSPI::set_format_spi(uint32_t clock, uint8_t bitOrder, uint8_t dataMode)
{
  SPISettings(clock, bitOrder, dataMode);
}

void TelemetrixSPI::spi_cs_control(uint8_t csPin, uint8_t csState)
{
  digitalWrite(csPin, csState);
}
