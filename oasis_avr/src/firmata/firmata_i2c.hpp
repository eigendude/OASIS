/*
 *  Copyright (C) 2021-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  This file is derived from the FirmataExpress project and the AGPL-3. License
 *  Copyright (C) 2006-2008 Hans-Christoph Steiner. All rights reserved.
 *  Copyright (C) 2009-2017 Jeff Hoefs. All rights reserved.
 *  Copyright (C) 2018-2019 Alan Yorinks. All Rights Reserved.
 *
 *  SPDX-License-Identifier: Apache-2.0 AND AGPL-3
 *  See DOCS/LICENSING.md for more information.
 */
#pragma once

#include "firmata_subsystem.hpp"

#include <stdint.h>

#include <binary.h>

namespace OASIS
{

// I2C constants
// clang-format off
static constexpr uint8_t I2C_WRITE =                   B00000000;
static constexpr uint8_t I2C_READ =                    B00001000;
static constexpr uint8_t I2C_READ_CONTINUOUSLY =       B00010000;
static constexpr uint8_t I2C_STOP_READING =            B00011000;
static constexpr uint8_t I2C_READ_WRITE_MODE_MASK =    B00011000;
static constexpr uint8_t I2C_10BIT_ADDRESS_MODE_MASK = B00100000;
static constexpr uint8_t I2C_END_TX_MASK =             B01000000;
static constexpr uint8_t I2C_STOP_TX =                 1;
static constexpr uint8_t I2C_RESTART_TX =              0;
static constexpr uint8_t I2C_MAX_QUERIES =             8;
static constexpr uint8_t I2C_REGISTER_NOT_SPECIFIED =  -1;
// clang-format on

class FirmataI2C : public FirmataSubsystem
{
public:
  // Implementation of FirmataSubsystem
  void Loop() override;

  // I2C functions
  void SetI2CMode(uint8_t digitalPin);
  void EnableI2CPins();
  /*!
   * \brief Disable the I2C pins so they can be used for other functions
   */
  void DisableI2CPins();
  bool IsI2CEnabled() const { return m_isI2CEnabled; }
  void ReadAndReportI2CData(uint8_t address, int theRegister, uint8_t numBytes, uint8_t stopTX);
  unsigned int GetI2CQueryCount() const;
  uint8_t GetI2CQueryAddress(uint8_t queryIndex) const;
  void AddI2CQuery(uint8_t slaveAddress, int slaveRegister, uint8_t data, uint8_t stopTX);
  void RemoveI2CQuery(uint8_t queryIndex);
  void SetPreviousI2CQuery();
  void DisableI2CReporting();
  void SetI2CReadDelayTime(unsigned int delayTime);

private:
  // I2C types
  /*!
   * \brief I2C data
   */
  struct i2c_device_info
  {
    uint8_t addr;
    int reg;
    uint8_t bytes;
    uint8_t stopTX;
  };

  // I2C state
  // For I2C read continuous mode
  i2c_device_info m_query[I2C_MAX_QUERIES]{};
  uint8_t m_i2cRxData[64]{};
  bool m_isI2CEnabled = false;
  int8_t m_queryIndex = -1;
  // Default delay time between I2C read request and Wire.requestFrom()
  unsigned int m_i2cReadDelayTime = 0;
};

} // namespace OASIS
