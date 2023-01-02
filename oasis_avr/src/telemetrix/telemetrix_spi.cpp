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

void TelemetrixSPI::InitSPI(uint8_t pinCount, const uint8_t* pins)
{
  if (m_spiReportMessage == nullptr)
    m_spiReportMessage = new uint8_t[64];

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

void TelemetrixSPI::WriteBlockingSPI(uint8_t byteCount, const uint8_t* data)
{
  for (unsigned int i = 0; i < byteCount; ++i)
    SPI.transfer(data[i]);
}

void TelemetrixSPI::ReadBlockingSPI(uint8_t byteCount, uint8_t readRegister)
{
  // m_spiReportMessage[0] = length of message including this element
  // m_spiReportMessage[1] = SPI_REPORT
  // m_spiReportMessage[2] = register used for the read
  // m_spiReportMessage[3] = number of bytes returned
  // m_spiReportMessage[4..] = data read

  // Configure the report message
  // Calculate the packet length
  m_spiReportMessage[0] = byteCount + 3; // Packet length
  m_spiReportMessage[1] = SPI_REPORT;
  m_spiReportMessage[2] = readRegister; // Register
  m_spiReportMessage[3] = byteCount; // Number of bytes read

  // Write the register out. OR it with 0x80 to indicate a read
  SPI.transfer(readRegister | 0x80);

  // Now read the specified number of bytes and place them in the report buffer
  for (unsigned int i = 0; i < byteCount; ++i)
    m_spiReportMessage[i + 4] = SPI.transfer(0x00);

  Serial.write(m_spiReportMessage, byteCount + 4);
}

void TelemetrixSPI::SetFormatSPI(uint32_t clock, uint8_t bitOrder, uint8_t dataMode)
{
  SPISettings(clock, bitOrder, dataMode);
}

void TelemetrixSPI::SPICSControl(uint8_t csPin, uint8_t csState)
{
  digitalWrite(csPin, csState);
}
