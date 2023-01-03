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

class TwoWire;

namespace OASIS
{
class TelemetrixI2C
{
public:
  void I2CBegin(uint8_t i2cPort);

  void I2CRead(uint8_t i2cPort,
               uint8_t address,
               uint8_t theRegister,
               uint8_t byteCount,
               bool stopTransmitting,
               bool writeByte);
  void I2CWrite(uint8_t i2cPort, uint8_t i2cAddress, uint8_t byteCount, const uint8_t* data);

private:
  // Get a pointer to the active TwoWire object for the given port
  TwoWire* GetI2CInstance(uint8_t i2cPort);
};
} // namespace OASIS
