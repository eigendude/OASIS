/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace OASIS::IMU
{
using Vec3 = std::array<double, 3>;
using Mat3 = std::array<std::array<double, 3>, 3>;

/*!
 * \brief One IMU sample in the IMU frame
 */
struct ImuSample
{
  /*!
   * \brief Linear acceleration
   *
   * Units: m/s^2
   */
  Vec3 accel_mps2{0.0, 0.0, 0.0};

  /*!
   * \brief Angular velocity
   *
   * Units: rad/s
   */
  Vec3 gyro_rads{0.0, 0.0, 0.0};

  /*!
   * \brief Per-sample measurement noise covariance for accel
   *
   * Units: (m/s^2)^2
   */
  Mat3 accel_cov_mps2_2{};

  /*!
   * \brief Per-sample measurement noise covariance for gyro
   *
   * Units: (rad/s)^2
   */
  Mat3 gyro_cov_rads2_2{};

  /*!
   * \brief Temperature at sample time
   *
   * Units: deg C
   */
  double temperature_c{0.0};

  /*!
   * \brief Temperature measurement variance at sample time
   *
   * Units: (deg C)^2
   */
  double temperature_var_c2{0.0};

  /*!
   * \brief Monotonic timestamp for dt computations
   *
   * Units: seconds
   */
  double stamp_s{0.0};
};

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
} // namespace OASIS::IMU
