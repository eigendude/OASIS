/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "imu/mpu6050/Mpu6050ImuCalibrationTypes.h"

#include <filesystem>
#include <string>

namespace OASIS::IMU
{
/*!
 * \brief Loads and saves IMU calibration to disk
 *
 * File I/O is kept ROS-free so nodes can stay thin.
 */
class ImuCalibrationFile
{
public:
  bool Load(const std::filesystem::path& path, ImuCalibrationRecord& out) const;
  bool Save(const std::filesystem::path& path, const ImuCalibrationRecord& rec) const;

  static std::string DefaultFilename();
  static constexpr int Version() { return 2; }
};
} // namespace OASIS::IMU
