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

// Uncomment out the next line to create a second I2C port
//#define SECOND_I2C_PORT

// Change the pins to match SDA and SCL for your board
#if defined(SECOND_I2C_PORT)
#define SECOND_I2C_PORT_SDA PB3
#define SECOND_I2C_PORT_SCL PB10
#endif

class TwoWire;

namespace OASIS
{
class TelemetrixI2C
{
public:
  TelemetrixI2C();

  void I2CBegin(uint8_t i2cPort);
  void I2CRead(uint8_t address,
               uint8_t theRegister,
               uint8_t byteCount,
               bool stopTransmitting,
               uint8_t i2cPort,
               bool writeByte);
  void I2CWrite(uint8_t byteCount, uint8_t deviceAddress, uint8_t i2cPort, const uint8_t* data);

private:
  // A buffer to hold i2c report data
  uint8_t* m_i2cReportMessage{nullptr};

  // Optional second I2C port
  TwoWire* m_wire2{nullptr};

  // A pointer to an active TwoWire object
  TwoWire* m_currentI2CPort{nullptr};
};
} // namespace OASIS
