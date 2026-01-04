/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "imu/ImuTypes.h"

#include <cstddef>
#include <vector>

namespace OASIS::IMU
{
/*!
 * \brief Windowed stationary detector for IMU calibration gating
 *
 * Intended to reject motion by checking variance and gyro mean in a short window
 */
class StationaryDetector
{
public:
  struct Config
  {
    /*!
     * \brief Sliding window size
     *
     * Units: samples
     */
    std::size_t window_size{20};

    /*!
     * \brief Variance gate multiplier on per-axis noise variance
     *
     * Units: unitless
     */
    double variance_sigma_mult{3.0};

    /*!
     * \brief Gyro mean gate multiplier on mean sigma
     *
     * Units: unitless
     */
    double gyro_mean_sigma_mult{3.0};

    /*!
     * \brief Minimum mean-sigma floor for gyro gate
     *
     * Units: rad/s
     */
    double gyro_sigma_mean_floor_rads{0.002};
  };

  struct Noise
  {
    /*!
     * \brief Expected accel noise covariance
     *
     * Units: (m/s^2)^2
     */
    Mat3 accel_cov_mps2_2{};

    /*!
     * \brief Expected gyro noise covariance
     *
     * Units: (rad/s)^2
     */
    Mat3 gyro_cov_rads2_2{};
  };

  struct Status
  {
    /*!
     * \brief True when window is classified as stationary
     */
    bool stationary{false};

    /*!
     * \brief Mean accel over window
     *
     * Units: m/s^2
     */
    Vec3 mean_accel_mps2{0.0, 0.0, 0.0};

    /*!
     * \brief Mean gyro over window
     *
     * Units: rad/s
     */
    Vec3 mean_gyro_rads{0.0, 0.0, 0.0};
  };

  StationaryDetector() = default;

  void Configure(const Config& cfg);
  void Reset();

  Status Update(const ImuSample& sample, const Noise& noise);

private:
  Config m_cfg{};

  std::vector<ImuSample> m_window;

  Vec3 m_gyro_bias_iir{0.0, 0.0, 0.0};
  bool m_gyro_bias_iir_init{false};
};
} // namespace OASIS::IMU
