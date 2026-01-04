/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <string>

namespace OASIS
{
namespace Magnetometer
{

enum class MeasurementMode
{
  // Meaning: force SET before measurement
  Set,

  // Meaning: force RESET before measurement
  Reset
};

struct Mmc5983maConfig
{
  // Meaning: I2C device path
  std::string i2c_device;

  // Meaning: 7-bit I2C address
  std::uint8_t i2c_address{0};

  // Meaning: raw rate register code for Control2
  std::uint8_t raw_rate_code{0};

  // Meaning: bandwidth mode selector (datasheet BW=00..11)
  std::uint8_t bandwidth_mode{0};

  // Units: Tesla
  // Meaning: conversion factor from raw counts to Tesla
  double tesla_per_count{0.0};

  // Units: milliseconds
  // Meaning: maximum time to wait for measurement completion
  int measurement_timeout_ms{0};
};

class Mmc5983maDevice
{
public:
  bool Initialize(const Mmc5983maConfig& config);
  bool ReadProductId(std::uint8_t& product_id) const;
  bool TakeMeasurement(MeasurementMode mode, Eigen::Vector3d& sample_t) const;

private:
  bool WriteRegister(std::uint8_t reg, std::uint8_t value) const;
  bool ReadRegister(std::uint8_t reg, std::uint8_t& value) const;
  bool ReadRegisters(std::uint8_t reg, std::uint8_t* values, std::size_t length) const;
  bool WaitForMeasurementDone() const;
  Eigen::Vector3d ParseSample(const std::uint8_t* buffer) const;

  Mmc5983maConfig m_config;
};

} // namespace Magnetometer
} // namespace OASIS
