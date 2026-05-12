/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "imu/ImuTypes.h"

namespace OASIS::IMU
{
/*!
 * \brief One MPU6050 IMU sample in the IMU frame
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
} // namespace OASIS::IMU
