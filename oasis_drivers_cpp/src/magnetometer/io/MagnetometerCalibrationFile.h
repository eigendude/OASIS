/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>

#include <filesystem>
#include <cstdint>
#include <string>

namespace OASIS
{
namespace Magnetometer
{

struct MagnetometerCalibrationRecord
{
  // Units: seconds
  // Meaning: timestamp for calibration record
  double timestamp_s{0.0};

  // Meaning: bandwidth mode setting used for sampling
  std::uint8_t bandwidth_mode{0};

  // Units: Hz
  // Meaning: raw measurement rate configured
  std::uint16_t raw_rate_hz{0};

  // Units: Tesla^2
  // Meaning: covariance matrix estimate
  Eigen::Matrix3d covariance_t2 = Eigen::Matrix3d::Identity();

  // Units: Tesla
  // Meaning: last computed offset vector for diagnostics
  Eigen::Vector3d offset_t = Eigen::Vector3d::Zero();
};

class MagnetometerCalibrationFile
{
public:
  bool Write(const std::filesystem::path& path,
             const MagnetometerCalibrationRecord& record) const;
};

} // namespace Magnetometer
} // namespace OASIS
