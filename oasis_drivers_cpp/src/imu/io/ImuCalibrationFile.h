/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "imu/ImuTypes.h"

#include <cstdint>
#include <filesystem>
#include <string>

namespace OASIS::IMU
{
/*!
 * \brief Ellipsoid fit for raw accelerometer samples
 *
 * Quadratic form:
 *   (x - c)^T * Q * (x - c) = 1
 *
 * Units:
 * - x, c: m/s^2
 * - Q: 1/(m/s^2)^2
 */
struct AccelEllipsoid
{
  /*!
   * \brief Ellipsoid center c
   *
   * Units: m/s^2
   */
  Vec3 center_mps2{0.0, 0.0, 0.0};

  /*!
   * \brief Quadratic form matrix Q
   *
   * Units: 1/(m/s^2)^2
   * Layout: row-major 3x3
   */
  Mat3 Q{};
};

/*!
 * \brief Complete IMU calibration record stored on disk
 *
 * This wraps systematic calibration, measured measurement noise, and metadata.
 */
struct ImuCalibrationRecord
{
  /*!
   * \brief File creation timestamp
   *
   * Units: Unix time in nanoseconds
   */
  std::uint64_t created_unix_ns{0};

  /*!
   * \brief Gravity used during calibration and scaling
   *
   * Units: m/s^2
   */
  double gravity_mps2{9.80665};

  /*!
   * \brief Number of samples used in the fit
   *
   * Units: samples
   */
  std::uint32_t fit_sample_count{0};

  /*!
   * \brief Raw and corrected measurement noise used for sensor_msgs/Imu covariances
   */
  ImuMeasurementNoise measurement_noise{};

  /*!
   * \brief Raw accelerometer ellipsoid fit
   */
  AccelEllipsoid accel_ellipsoid{};

  /*!
   * \brief Systematic calibration parameters and uncertainty
   */
  ImuCalibration calib{};
};

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
