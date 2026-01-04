/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "imu/ImuTypes.h"

#include <cstddef>

#include <Eigen/Dense>

namespace OASIS::IMU
{
/*!
 * \brief Estimates constant gyroscope bias using fused stationary windows.
 *
 * This estimator fuses many stationary-window measurements of the mean gyro rate
 * into a single bias estimate with a full 3×3 covariance, using a Kalman-style
 * measurement update (equivalent to sequential Bayesian fusion of Gaussians).
 *
 * Model (during stationary):
 *   z = b_g + v
 *
 * Where:
 *   z   : measured mean angular rate over a stationary window (rad/s)
 *   b_g : constant gyroscope bias (rad/s)
 *   v   : zero-mean measurement noise for the window mean
 *
 * The input covariance is the *sample covariance over the window* (units (rad/s)^2).
 * Internally, the update treats the measurement noise for the *mean* as:
 *
 *   R_mean = cov_sample / N
 *
 * where N is the window sample count (window_count). This assumes the samples in the
 * window are approximately independent (or that cov_sample already reflects the
 * effective correlation). If the IMU has strong colored noise, N may overstate the
 * reduction—StationaryDetector gating plus covariance floors/jitter help keep it stable.
 *
 * Notes:
 * - No process model is applied (i.e., bias is assumed constant between updates).
 * - This is effectively a recursive weighted least squares estimator when starting
 *   from an uninformative prior.
 * - All matrices are symmetrized and a small diagonal jitter is added in the .cpp
 *   to avoid singular inversions.
 */
class GyroBiasEstimator
{
public:
  /*!
   * \brief Reset estimator state.
   *
   * After Reset(), the estimator is uninitialized. The next Update() call will
   * initialize the bias to the provided measurement and covariance.
   */
  void Reset();

  /*!
   * \brief Update estimator using a stationary-window gyro measurement.
   *
   * \param z_mean_rads
   *   Mean gyro rate over the stationary window.
   *   Units: rad/s
   *
   * \param cov_sample_rads2_2
   *   Sample covariance of gyro samples within the window (not divided by N).
   *   Units: (rad/s)^2
   *   Layout: row-major 3×3
   *
   * \param window_count
   *   Number of samples used to compute z_mean_rads and cov_sample_rads2_2.
   *   Units: samples
   *
   * Behavior:
   * - Converts inputs to Eigen.
   * - Computes measurement covariance for the mean: R = cov_sample / window_count.
   * - If uninitialized: sets b := z_mean, P := R (prior from first measurement).
   * - Otherwise performs a Kalman measurement update with H = I and no process noise:
   *     S = P + R
   *     K = P * S^{-1}
   *     b = b + K * (z - b)
   *     P = (I-K) P (I-K)^T + K R K^T   (Joseph form for numerical stability)
   */
  void Update(const Vec3& z_mean_rads, const Mat3& cov_sample_rads2_2, std::size_t window_count);

  /*!
   * \brief True if at least one measurement has been fused.
   */
  bool IsInitialized() const;

  /*!
   * \brief Return current bias estimate b_g.
   *
   * Units: rad/s
   */
  Vec3 GetBias() const;

  /*!
   * \brief Return current bias covariance P.
   *
   * Units: (rad/s)^2
   * Layout: row-major 3×3
   */
  Mat3 GetCov() const;

private:
  /*!
   * \brief True after the first successful Update().
   */
  bool m_initialized{false};

  /*!
   * \brief Current bias estimate b_g in Eigen form.
   *
   * Units: rad/s
   */
  Eigen::Vector3d m_b = Eigen::Vector3d::Zero();

  /*!
   * \brief Current bias covariance P in Eigen form.
   *
   * Units: (rad/s)^2
   */
  Eigen::Matrix3d m_P = Eigen::Matrix3d::Zero();

  /*!
   * \brief Convert Vec3 (std::array) to Eigen vector.
   */
  static Eigen::Vector3d ToEigen(const Vec3& v);

  /*!
   * \brief Convert Mat3 (row-major std::array) to Eigen matrix.
   */
  static Eigen::Matrix3d ToEigen(const Mat3& m);

  /*!
   * \brief Convert Eigen vector to Vec3 (std::array).
   */
  static Vec3 FromEigen(const Eigen::Vector3d& v);

  /*!
   * \brief Convert Eigen matrix to Mat3 (row-major std::array).
   */
  static Mat3 FromEigen(const Eigen::Matrix3d& m);
};
} // namespace OASIS::IMU
