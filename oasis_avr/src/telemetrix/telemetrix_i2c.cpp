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

void TelemetrixI2C::i2c_begin(uint8_t i2c_port)
{
  if (i2c_port == 0)
    Wire.begin();

#ifdef SECOND_I2C_PORT
  else
    m_wire2->begin();
#endif
}

void TelemetrixI2C::i2c_read(uint8_t address,
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
    current_i2c_port = &Wire;

#ifdef SECOND_I2C_PORT
  // This is for port 2
  if (i2cPort == 1)
    current_i2c_port = m_wire2;
#endif

  // Write byte is true, then write the register
  if (writeByte)
  {
    current_i2c_port->beginTransmission(address);
    current_i2c_port->write(theRegister);
    current_i2c_port->endTransmission(stopTransmitting ? 1 : 0); // default = true
  }
  current_i2c_port->requestFrom(address, byteCount); // All bytes are returned in requestFrom

  // Check to be sure correct number of bytes were returned by slave
  if (byteCount < current_i2c_port->available())
  {
    const uint8_t report_message[4] = {3, I2C_TOO_FEW_BYTES_RCVD, 1, address};
    Serial.write(report_message, 4);
    return;
  }
  else if (byteCount > current_i2c_port->available())
  {
    const uint8_t report_message[4] = {3, I2C_TOO_MANY_BYTES_RCVD, 1, address};
    Serial.write(report_message, 4);
    return;
  }

  // Packet length
  i2c_report_message[0] = byteCount + 5;

  // Report type
  i2c_report_message[1] = I2C_READ_REPORT;

  // I2C_port
  i2c_report_message[2] = i2cPort;

  // Number of bytes read
  i2c_report_message[3] = byteCount; // number of bytes

  // Device address
  i2c_report_message[4] = address;

  // Device register
  i2c_report_message[5] = theRegister;

  // Append the data that was read
  unsigned int message_size = 0;
  for (; message_size < byteCount && current_i2c_port->available(); ++message_size)
    i2c_report_message[6 + message_size] = current_i2c_port->read();

  // Send slave address, register and received bytes
  for (unsigned int i = 0; i < message_size + 6; ++i)
    Serial.write(i2c_report_message[i]);
}

void TelemetrixI2C::i2c_write(uint8_t byteCount,
                              uint8_t deviceAddress,
                              uint8_t i2cPort,
                              const uint8_t* data)
{
  // Set the current i2c port if this is for the primary i2c
  if (i2cPort == 0)
    current_i2c_port = &Wire;

#ifdef SECOND_I2C_PORT
  // This is for port 2
  if (i2cPort == 1)
    current_i2c_port = m_wire2;
#endif

  current_i2c_port->beginTransmission(deviceAddress);

  // Write the data to the device
  for (unsigned int i = 0; i < byteCount; ++i)
    current_i2c_port->write(data[i]);

  current_i2c_port->endTransmission();
  delayMicroseconds(70);
}
