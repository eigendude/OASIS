/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "imu/ImuTypes.h"

#include <array>
#include <cstdint>

namespace OASIS::IMU
{
/*!
 * \brief Measured short-term measurement-noise statistics
 *
 * Intended for sensor_msgs/Imu covariance fields (per-sample noise), not the
 * systematic calibration parameter covariance.
 */
struct ImuMeasurementNoise
{
  /*!
   * \brief Accel measurement-noise covariance for raw measurements
   *
   * Units: (m/s^2)^2
   * Layout: row-major 3x3
   * Raw measurements are published on the imu_raw topic
   */
  Mat3 accel_cov_raw_mps2_2{};

  /*!
   * \brief Gyro measurement-noise covariance for raw measurements
   *
   * Units: (rad/s)^2
   * Layout: row-major 3x3
   * Raw measurements are published on the imu_raw topic
   */
  Mat3 gyro_cov_raw_rads2_2{};

  /*!
   * \brief Accel measurement-noise covariance for corrected measurements
   *
   * Units: (m/s^2)^2
   * Layout: row-major 3x3
   * Corrected measurements are published on the imu topic
   */
  Mat3 accel_cov_corrected_mps2_2{};

  /*!
   * \brief Gyro measurement-noise covariance for corrected measurements
   *
   * Units: (rad/s)^2
   * Layout: row-major 3x3
   * Corrected measurements are published on the imu topic
   */
  Mat3 gyro_cov_corrected_rads2_2{};
};

/*!
 * \brief IMU calibration parameters and systematic uncertainty
 *
 * Model
 * - Accelerometer: a_cal = A * (a_raw - b_a)
 * - Gyroscope:     w_cal = w_raw - b_g
 */
struct ImuCalibration
{
  /*!
   * \brief Accelerometer bias b_a
   *
   * Units: m/s^2
   */
  Vec3 accel_bias_mps2{0.0, 0.0, 0.0};

  /*!
   * \brief Accelerometer correction matrix A
   *
   * Units: unitless
   * Layout: row-major 3x3
   */
  Mat3 accel_A{{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}};

  /*!
   * \brief Covariance of [b_a, vec(A)] parameters
   *
   * Units:
   * - b_a terms: (m/s^2)^2
   * - A terms:   unitless^2
   *
   * Params: [bx, by, bz, a00, a01, a02, a10, a11, a12, a20, a21, a22]
   * Layout: row-major 12x12
   */
  std::array<std::array<double, 12>, 12> accel_param_cov{};

  /*!
   * \brief Fit-quality metric
   *
   * Units: m/s^2
   */
  double rms_residual_mps2{0.0};

  /*!
   * \brief Gyro bias b_g
   *
   * Units: rad/s
   */
  Vec3 gyro_bias_rads{0.0, 0.0, 0.0};

  /*!
   * \brief Covariance of gyro bias estimate
   *
   * Units: (rad/s)^2
   * Layout: row-major 3x3
   */
  Mat3 gyro_bias_cov_rads2_2{};

  /*!
   * \brief Mean temperature of samples used to estimate this calibration
   *
   * Units: deg C
   */
  double temperature_c{0.0};

  /*!
   * \brief Temperature variance of samples used to estimate this calibration
   *
   * Units: (deg C)^2
   */
  double temperature_var_c2{0.0};

  /*!
   * \brief True when calibration is populated and safe to apply
   */
  bool valid{false};
};

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
   * \brief Raw and corrected measurement noise used for sensor_msgs/Imu
   * covariances
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
} // namespace OASIS::IMU
