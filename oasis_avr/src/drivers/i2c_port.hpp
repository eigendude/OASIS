/*
 *  Copyright (C) 2022-2023 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  This file is derived from Telemetrix4Arduino under the AGPL 3.0 License
 *  Copyright (c) 2020-2021 Alan Yorinks
 *
 *  SPDX-License-Identifier: Apache-2.0 AND AGPL-3.0
 *  See DOCS/LICENSING.md for more information.
 */

#pragma once

#include "i2c_device_types.hpp"
#include "utils/timer.hpp"

#include <stdint.h>

// TODO
#define MAX_I2C_DEVICES 6

class TwoWire;

namespace OASIS
{
class I2CPort
{
private:
  I2CPort(uint8_t i2cPortIndex, TwoWire& i2cInstance);

public:
  static I2CPort* CreateI2CPort(uint8_t i2cPortIndex);

  ~I2CPort();

  uint8_t I2CPortIndex() const { return m_i2cPortIndex; }

  int I2CRead(uint8_t address,
              uint8_t theRegister,
              uint8_t byteCount,
              bool stopTransmitting,
              bool writeByte,
              uint8_t* i2cData);
  void I2CWrite(uint8_t i2cAddress, uint8_t byteCount, const uint8_t* data);

  /*!
   * \brief Begin communicating with a CCS811 air quality sensor
   *
   * Default I2C address is 0x5B. Alternate I2C address is 0x5A.
   */
  void BeginCCS811(uint8_t i2cAddress);
  void EndCCS811(uint8_t i2cAddress);
  void ScanCCS811Sensors(CCS811ScanCallback scanCallback);

  /*!
   * \brief Begin communicating with an MPU6050 IMU sensor
   *
   * Default I2C address is 0x68 (address pin low). Alternative I2C address is
   * 0x69 (address pin high).
   */
  void BeginMPU6050(uint8_t i2cAddress);
  void EndMPU6050(uint8_t i2cAddress);
  void ScanMPU6050Sensors(MPU6050ScanCallback scanCallback);

private:
  struct I2CDevice
  {
    const I2CDeviceType deviceType;
    const uint8_t i2cAddress;
    const uint32_t samplingIntervalMs;
    void* const driver;
    Timer sampleTimer;
  };

  int GetNextDeviceIndex() const;

  // Construction parameters
  const uint8_t m_i2cPortIndex;
  TwoWire* const m_i2cInstance;

  // I2C devices
  I2CDevice* m_i2cDevices[MAX_I2C_DEVICES];
};
} // namespace OASIS
