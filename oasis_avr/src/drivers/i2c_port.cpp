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

#include "i2c_port.hpp"

#include <Arduino.h>
#include <Wire.h>
#include <avr/io.h>

#if defined(ENABLE_AIR_QUALITY)
#include <SparkFunCCS811.h>
#endif

#if defined(ENABLE_IMU)
#include <MPU6050.h>
#endif

// To enable second I2C port, uncomment and change the pins to match your board
//#define SECOND_I2C_PORT_SDA PB3
//#define SECOND_I2C_PORT_SCL PB10

using namespace OASIS;

namespace
{
constexpr const uint32_t CCS811_SCAN_INTERVAL_MS = 1000;
constexpr const uint32_t MPU6050_SAMPLE_INTERAL_MS = 100;
} // namespace

I2CPort* I2CPort::CreateI2CPort(uint8_t i2cPortIndex)
{
  TwoWire* i2cInstance = nullptr;

  // Check if this is for the primary I2C
  if (i2cPortIndex == 0)
    i2cInstance = &Wire;

  if (i2cPortIndex == 1)
  {
#if defined(SECOND_I2C_PORT_SDA) && defined(SECOND_I2C_PORT_SCL)
    i2cInstance = new TwoWire(SECOND_I2C_PORT_SDA, SECOND_I2C_PORT_SCL);
#endif
  }

  if (i2cInstance != nullptr)
    return new I2CPort(i2cPortIndex, *i2cInstance);

  return nullptr;
}

I2CPort::I2CPort(uint8_t i2cPortIndex, TwoWire& i2cInstance)
  : m_i2cPortIndex(i2cPortIndex), m_i2cInstance(&i2cInstance)
{
  m_i2cInstance->begin();
}

I2CPort::~I2CPort()
{
  if (m_i2cInstance != &Wire)
    delete m_i2cInstance;
}

int I2CPort::I2CRead(uint8_t i2cAddress,
                     uint8_t theRegister,
                     uint8_t byteCount,
                     bool stopTransmitting,
                     bool writeByte,
                     uint8_t* i2cData)
{
  // Write byte is true, then write the register
  if (writeByte)
  {
    m_i2cInstance->beginTransmission(i2cAddress);
    m_i2cInstance->write(theRegister);
    m_i2cInstance->endTransmission(stopTransmitting ? 1 : 0); // default = true
  }
  m_i2cInstance->requestFrom(i2cAddress, byteCount); // All bytes are returned in requestFrom

  // Check to be sure correct number of bytes were returned by slave
  if (byteCount == m_i2cInstance->available())
  {
    // Append the data that was read
    unsigned int messageSize = 0;
    for (; messageSize < byteCount && m_i2cInstance->available(); ++messageSize)
      i2cData[messageSize] = m_i2cInstance->read();
  }

  return m_i2cInstance->available();
}

void I2CPort::I2CWrite(uint8_t i2cAddress, uint8_t byteCount, const uint8_t* data)
{
  m_i2cInstance->beginTransmission(i2cAddress);

  // Write the data to the device
  for (unsigned int i = 0; i < byteCount; ++i)
    m_i2cInstance->write(data[i]);

  m_i2cInstance->endTransmission();

  delayMicroseconds(70);
}

void I2CPort::BeginCCS811(uint8_t i2cAddress)
{
#if defined(ENABLE_AIR_QUALITY)
  const int deviceIndex = GetNextDeviceIndex();
  if (deviceIndex >= 0)
  {
    CCS811* driver = new CCS811(i2cAddress);

    if (!driver->begin(*m_i2cInstance))
    {
      delete driver;
    }
    else
    {
      m_i2cDevices[deviceIndex] =
          new I2CDevice{I2CDeviceType::CCS811, i2cAddress, CCS811_SCAN_INTERVAL_MS, driver};
    }
  }
#endif
}

void I2CPort::EndCCS811(uint8_t i2cAddress)
{
#if defined(ENABLE_AIR_QUALITY)
  for (unsigned int i = 0; i < MAX_I2C_DEVICES; ++i)
  {
    I2CDevice* device = m_i2cDevices[i];
    if (device == nullptr)
      continue;

    if (device->deviceType != I2CDeviceType::CCS811 || device->i2cAddress != i2cAddress)
      continue;

    CCS811* driver = static_cast<CCS811*>(device->driver);

    delete driver;
    delete device;
    m_i2cDevices[i] = nullptr;
  }
#endif
}

void I2CPort::ScanCCS811Sensors(CCS811ScanCallback scanCallback)
{
#if defined(ENABLE_AIR_QUALITY)
  for (unsigned int i = 0; i < MAX_I2C_DEVICES; ++i)
  {
    I2CDevice* device = m_i2cDevices[i];
    if (device == nullptr || device->deviceType != I2CDeviceType::CCS811)
      continue;

    if (device->sampleTimer.IsExpired())
    {
      device->sampleTimer.SetTimeout(device->samplingIntervalMs);

      CCS811* driver = static_cast<CCS811*>(device->driver);
      if (driver->dataAvailable())
      {
        driver->readAlgorithmResults();

        const uint16_t co2Ppb = driver->getCO2();
        const uint16_t tvocPpb = driver->getTVOC();

        scanCallback(m_i2cPortIndex, device->i2cAddress, co2Ppb, tvocPpb);
      }
    }
  }
#endif
}

void I2CPort::BeginMPU6050(uint8_t i2cAddress)
{
#if defined(ENABLE_IMU)
  const int deviceIndex = GetNextDeviceIndex();
  if (deviceIndex >= 0)
  {
    MPU6050_Base* driver = new MPU6050_Base(i2cAddress, static_cast<void*>(m_i2cInstance));

    // Initialize IMU
    driver->initialize();

    if (!driver->testConnection())
    {
      delete driver;
    }
    else
    {
      m_i2cDevices[deviceIndex] =
          new I2CDevice{I2CDeviceType::MPU6050, i2cAddress, MPU6050_SAMPLE_INTERAL_MS, driver};
    }
  }
#endif
}

void I2CPort::EndMPU6050(uint8_t i2cAddress)
{
#if defined(ENABLE_IMU)
  for (unsigned int i = 0; i < MAX_I2C_DEVICES; ++i)
  {
    I2CDevice* device = m_i2cDevices[i];
    if (device == nullptr)
      continue;

    if (device->deviceType != I2CDeviceType::MPU6050 || device->i2cAddress != i2cAddress)
      continue;

    MPU6050_Base* driver = static_cast<MPU6050_Base*>(device->driver);

    delete driver;
    delete device;
    m_i2cDevices[i] = nullptr;
  }
#endif
}

void I2CPort::ScanMPU6050Sensors(MPU6050ScanCallback scanCallback)
{
#if defined(ENABLE_IMU)
  for (unsigned int i = 0; i < MAX_I2C_DEVICES; ++i)
  {
    I2CDevice* device = m_i2cDevices[i];
    if (device == nullptr || device->deviceType != I2CDeviceType::MPU6050)
      continue;

    if (device->sampleTimer.IsExpired())
    {
      device->sampleTimer.SetTimeout(device->samplingIntervalMs);

      MPU6050_Base* driver = static_cast<MPU6050_Base*>(device->driver);

      // Read raw accel/gyro measurements from device
      int16_t ax = 0;
      int16_t ay = 0;
      int16_t az = 0;
      int16_t gx = 0;
      int16_t gy = 0;
      int16_t gz = 0;
      driver->getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

      scanCallback(m_i2cPortIndex, device->i2cAddress, ax, ay, az, gx, gy, gz);
    }
  }
#endif
}

int I2CPort::GetNextDeviceIndex() const
{
  for (unsigned int i = 0; i < MAX_I2C_DEVICES; ++i)
  {
    if (m_i2cDevices[i] == nullptr)
      return i;
  }

  return -1;
}
