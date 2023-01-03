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

void TelemetrixI2C::I2CBegin(uint8_t i2cPort)
{
  TwoWire* twoWire = GetI2CInstance(i2cPort);
  if (twoWire == nullptr)
    return;

  twoWire->begin();
}

void TelemetrixI2C::I2CRead(uint8_t i2cPort,
                            uint8_t i2cAddress,
                            uint8_t theRegister,
                            uint8_t byteCount,
                            bool stopTransmitting,
                            bool writeByte)
{
  TwoWire* twoWire = GetI2CInstance(i2cPort);
  if (twoWire == nullptr)
    return;

  // Data in the incoming message:
  // address, [0]
  // register, [1]
  // number of bytes, [2]
  // stop transmitting flag [3]
  // i2c port [4]
  // write the register [5]

  // Write byte is true, then write the register
  if (writeByte)
  {
    twoWire->beginTransmission(i2cAddress);
    twoWire->write(theRegister);
    twoWire->endTransmission(stopTransmitting ? 1 : 0); // default = true
  }
  twoWire->requestFrom(i2cAddress, byteCount); // All bytes are returned in requestFrom

  // Check to be sure correct number of bytes were returned by slave
  if (byteCount < twoWire->available())
  {
    const uint8_t report_message[4] = {3, I2C_TOO_FEW_BYTES_RCVD, 1, i2cAddress};
    Serial.write(report_message, 4);
    return;
  }
  else if (byteCount > twoWire->available())
  {
    const uint8_t report_message[4] = {3, I2C_TOO_MANY_BYTES_RCVD, 1, i2cAddress};
    Serial.write(report_message, 4);
    return;
  }

  uint8_t i2cReportMessage[64];

  // Packet length
  i2cReportMessage[0] = byteCount + 5;

  // Report type
  i2cReportMessage[1] = I2C_READ_REPORT;

  // I2C_port
  i2cReportMessage[2] = i2cPort;

  // Number of bytes read
  i2cReportMessage[3] = byteCount; // number of bytes

  // Device address
  i2cReportMessage[4] = i2cAddress;

  // Device register
  i2cReportMessage[5] = theRegister;

  // Append the data that was read
  unsigned int message_size = 0;
  for (; message_size < byteCount && twoWire->available(); ++message_size)
    i2cReportMessage[6 + message_size] = twoWire->read();

  // Send slave address, register and received bytes
  for (unsigned int i = 0; i < message_size + 6; ++i)
    Serial.write(i2cReportMessage[i]);
}

void TelemetrixI2C::I2CWrite(uint8_t i2cPort,
                             uint8_t i2cAddress,
                             uint8_t byteCount,
                             const uint8_t* data)
{

  TwoWire* twoWire = GetI2CInstance(i2cPort);
  if (twoWire == nullptr)
    return;

  twoWire->beginTransmission(i2cAddress);

  // Write the data to the device
  for (unsigned int i = 0; i < byteCount; ++i)
    twoWire->write(data[i]);

  twoWire->endTransmission();

  delayMicroseconds(70);
}

TwoWire* TelemetrixI2C::GetI2CInstance(uint8_t i2cPort)
{
  // Check if this is for the primary I2C
  if (i2cPort == 0)
    return &Wire;

#if defined(SECOND_I2C_PORT_SCL)
  if (i2cPort == 1)
  {
    // TODO
    static TwoWire wire2(PB3, PB10);
    return &wire2;
  }
#endif

  return nullptr;
}
