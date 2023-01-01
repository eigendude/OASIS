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

#include "telemetrix_i2c.hpp"

#include "telemetrix_reports.hpp"

#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>

using namespace OASIS;

TelemetrixI2C::TelemetrixI2C()
{
#ifdef SECOND_I2C_PORT
  static TwoWire wire2(SECOND_I2C_PORT_SDA, SECOND_I2C_PORT_SCL);
  m_wire2 = &wire2;
#endif
}

void TelemetrixI2C::I2CBegin(uint8_t i2cPort)
{
  if (m_i2cReportMessage == nullptr)
    m_i2cReportMessage = new uint8_t[64];

  if (i2cPort == 0)
    Wire.begin();

#ifdef SECOND_I2C_PORT
  else
    m_wire2->begin();
#endif
}

void TelemetrixI2C::I2CRead(uint8_t address,
                            uint8_t theRegister,
                            uint8_t byteCount,
                            bool stopTransmitting,
                            uint8_t i2cPort,
                            bool writeByte)
{
  // Data in the incoming message:
  // address, [0]
  // register, [1]
  // number of bytes, [2]
  // stop transmitting flag [3]
  // i2c port [4]
  // write the register [5]

  // Set the current i2c port if this is for the primary i2c
  if (i2cPort == 0)
    m_currentI2CPort = &Wire;

#ifdef SECOND_I2C_PORT
  // This is for port 2
  if (i2cPort == 1)
    m_currentI2CPort = m_wire2;
#endif

  // Write byte is true, then write the register
  if (writeByte)
  {
    m_currentI2CPort->beginTransmission(address);
    m_currentI2CPort->write(theRegister);
    m_currentI2CPort->endTransmission(stopTransmitting ? 1 : 0); // default = true
  }
  m_currentI2CPort->requestFrom(address, byteCount); // All bytes are returned in requestFrom

  // Check to be sure correct number of bytes were returned by slave
  if (byteCount < m_currentI2CPort->available())
  {
    const uint8_t report_message[4] = {3, I2C_TOO_FEW_BYTES_RCVD, 1, address};
    Serial.write(report_message, 4);
    return;
  }
  else if (byteCount > m_currentI2CPort->available())
  {
    const uint8_t report_message[4] = {3, I2C_TOO_MANY_BYTES_RCVD, 1, address};
    Serial.write(report_message, 4);
    return;
  }

  // Packet length
  m_i2cReportMessage[0] = byteCount + 5;

  // Report type
  m_i2cReportMessage[1] = I2C_READ_REPORT;

  // I2C_port
  m_i2cReportMessage[2] = i2cPort;

  // Number of bytes read
  m_i2cReportMessage[3] = byteCount; // number of bytes

  // Device address
  m_i2cReportMessage[4] = address;

  // Device register
  m_i2cReportMessage[5] = theRegister;

  // Append the data that was read
  unsigned int message_size = 0;
  for (; message_size < byteCount && m_currentI2CPort->available(); ++message_size)
    m_i2cReportMessage[6 + message_size] = m_currentI2CPort->read();

  // Send slave address, register and received bytes
  for (unsigned int i = 0; i < message_size + 6; ++i)
    Serial.write(m_i2cReportMessage[i]);
}

void TelemetrixI2C::I2CWrite(uint8_t byteCount,
                             uint8_t deviceAddress,
                             uint8_t i2cPort,
                             const uint8_t* data)
{
  // Set the current i2c port if this is for the primary i2c
  if (i2cPort == 0)
    m_currentI2CPort = &Wire;

#ifdef SECOND_I2C_PORT
  // This is for port 2
  if (i2cPort == 1)
    m_currentI2CPort = m_wire2;
#endif

  m_currentI2CPort->beginTransmission(deviceAddress);

  // Write the data to the device
  for (unsigned int i = 0; i < byteCount; ++i)
    m_currentI2CPort->write(data[i]);

  m_currentI2CPort->endTransmission();
  delayMicroseconds(70);
}
