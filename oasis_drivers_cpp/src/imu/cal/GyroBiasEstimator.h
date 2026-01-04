/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "imu/ImuTypes.h"

#include <Eigen/Dense>

#include <cstddef>

namespace OASIS::IMU
{
class GyroBiasEstimator
{
public:
  void Reset();

  /*!
   * \brief Update with a stationary-window gyro measurement
   *
   * \param z_mean_rads Mean gyro over window
   * \param cov_sample_rads2_2 Sample covariance over window
   * \param window_count Sample count in window
   */
  void Update(const Vec3& z_mean_rads,
              const Mat3& cov_sample_rads2_2,
              std::size_t window_count);

  bool IsInitialized() const;
  Vec3 GetBias() const;
  Mat3 GetCov() const;

private:
  bool m_initialized{false};
  Eigen::Vector3d m_b = Eigen::Vector3d::Zero();
  Eigen::Matrix3d m_P = Eigen::Matrix3d::Zero();

  static Eigen::Vector3d ToEigen(const Vec3& v);
  static Eigen::Matrix3d ToEigen(const Mat3& m);
  static Vec3 FromEigen(const Eigen::Vector3d& v);
  static Mat3 FromEigen(const Eigen::Matrix3d& m);
};
} // namespace OASIS::IMU
