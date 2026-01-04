/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "magnetometer/Mmc5983maDevice.h"

#include <chrono>
#include <thread>

#include <I2Cdev.h>

namespace OASIS
{
namespace Magnetometer
{

namespace
{

// I2C registers
constexpr std::uint8_t kRegXout0 = 0x00;
constexpr std::uint8_t kRegStatus = 0x08;
constexpr std::uint8_t kRegControl0 = 0x09;
constexpr std::uint8_t kRegControl1 = 0x0A;
constexpr std::uint8_t kRegControl2 = 0x0B;
constexpr std::uint8_t kRegControl3 = 0x0C;
constexpr std::uint8_t kRegProductId = 0x2F;

// Status flags
constexpr std::uint8_t kStatusMeasurementDone = 0x01;

// Control0 commands
constexpr std::uint8_t kControl0TakeMeasurement = 0x01;
constexpr std::uint8_t kControl0Set = 0x08;
constexpr std::uint8_t kControl0Reset = 0x10;

// Control1 fields
constexpr std::uint8_t kControl1BandwidthMask = 0x03;

// Control2 fields
constexpr std::uint8_t kControl2ContinuousEnable = 0x08;
constexpr std::uint8_t kControl2RateMask = 0x07;

// Units: counts
// Meaning: midpoint for 18-bit unsigned output
constexpr std::int32_t kRawMidpoint = 1 << 17;
} // namespace

bool Mmc5983maDevice::Initialize(const Mmc5983maConfig& config)
{
  m_config = config;

  I2Cdev::initialize(m_config.i2c_device.c_str());

  std::uint8_t control1 = 0;
  control1 |= (m_config.bandwidth_mode & kControl1BandwidthMask);
  if (!WriteRegister(kRegControl1, control1))
    return false;

  std::uint8_t control2 = 0;
  control2 &= ~kControl2ContinuousEnable;
  control2 |= (m_config.raw_rate_code & kControl2RateMask);
  if (!WriteRegister(kRegControl2, control2))
    return false;

  if (!WriteRegister(kRegControl3, 0x00))
    return false;

  return true;
}

bool Mmc5983maDevice::ReadProductId(std::uint8_t& product_id) const
{
  return ReadRegister(kRegProductId, product_id);
}

bool Mmc5983maDevice::TakeMeasurement(MeasurementMode mode, Eigen::Vector3d& sample_t) const
{
  const std::uint8_t setReset = mode == MeasurementMode::Set ? kControl0Set : kControl0Reset;
  if (!WriteRegister(kRegControl0, setReset))
    return false;

  if (!WriteRegister(kRegControl0, kControl0TakeMeasurement))
    return false;

  if (!WaitForMeasurementDone())
    return false;

  std::uint8_t buffer[7] = {};
  if (!ReadRegisters(kRegXout0, buffer, sizeof(buffer)))
    return false;

  sample_t = ParseSample(buffer);
  return true;
}

bool Mmc5983maDevice::WriteRegister(std::uint8_t reg, std::uint8_t value) const
{
  return I2Cdev::writeByte(m_config.i2c_address, reg, value);
}

bool Mmc5983maDevice::ReadRegister(std::uint8_t reg, std::uint8_t& value) const
{
  value = 0;
  const int result = I2Cdev::readBytes(m_config.i2c_address, reg, 1, &value);
  return result == 1;
}

bool Mmc5983maDevice::ReadRegisters(std::uint8_t reg,
                                    std::uint8_t* values,
                                    std::size_t length) const
{
  const int result =
      I2Cdev::readBytes(m_config.i2c_address, reg, static_cast<std::uint8_t>(length), values);
  return result == static_cast<int>(length);
}

bool Mmc5983maDevice::WaitForMeasurementDone() const
{
  const auto start = std::chrono::steady_clock::now();
  const auto timeout = std::chrono::milliseconds(m_config.measurement_timeout_ms);

  while (std::chrono::steady_clock::now() - start < timeout)
  {
    std::uint8_t status = 0;
    if (!ReadRegister(kRegStatus, status))
      return false;

    if ((status & kStatusMeasurementDone) != 0)
      return true;

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  return false;
}

Eigen::Vector3d Mmc5983maDevice::ParseSample(const std::uint8_t* buffer) const
{
  const std::uint32_t x_raw = (static_cast<std::uint32_t>(buffer[0]) << 10) |
                              (static_cast<std::uint32_t>(buffer[1]) << 2) |
                              ((buffer[6] >> 6) & 0x03);
  const std::uint32_t y_raw = (static_cast<std::uint32_t>(buffer[2]) << 10) |
                              (static_cast<std::uint32_t>(buffer[3]) << 2) |
                              ((buffer[6] >> 4) & 0x03);
  const std::uint32_t z_raw = (static_cast<std::uint32_t>(buffer[4]) << 10) |
                              (static_cast<std::uint32_t>(buffer[5]) << 2) |
                              ((buffer[6] >> 2) & 0x03);

  const std::int32_t x_signed = static_cast<std::int32_t>(x_raw) - kRawMidpoint;
  const std::int32_t y_signed = static_cast<std::int32_t>(y_raw) - kRawMidpoint;
  const std::int32_t z_signed = static_cast<std::int32_t>(z_raw) - kRawMidpoint;

  return Eigen::Vector3d{x_signed * m_config.tesla_per_count, y_signed * m_config.tesla_per_count,
                         z_signed * m_config.tesla_per_count};
}

} // namespace Magnetometer
} // namespace OASIS
