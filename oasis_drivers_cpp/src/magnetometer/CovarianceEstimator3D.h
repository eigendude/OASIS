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

#include <cstddef>
#include <deque>

namespace OASIS
{
namespace Magnetometer
{

struct StationaryGateConfig
{
  // Units: Tesla
  // Meaning: per-axis stddev threshold for stationary detection
  double axis_std_threshold_t{0.0};

  // Units: Tesla
  // Meaning: magnitude stddev threshold for stationary detection
  double magnitude_std_threshold_t{0.0};
};

struct CovarianceEstimatorConfig
{
  // Units: samples
  // Meaning: number of samples in the stationary window
  std::size_t window_size{0};

  // Units: samples
  // Meaning: prior equivalent sample count for shrinkage
  double prior_strength_samples{0.0};

  // Units: seconds
  // Meaning: EWMA time constant for covariance smoothing
  double ewma_tau_sec{0.0};

  // Units: seconds
  // Meaning: update period for EWMA computation
  double dt_sec{0.0};

  // Units: Tesla^2
  // Meaning: prior covariance matrix for shrinkage
  Eigen::Matrix3d prior_covariance = Eigen::Matrix3d::Identity();

  StationaryGateConfig gate;
};

struct CovarianceUpdate
{
  // Meaning: true if the stationary gate accepted the window
  bool is_stationary{false};

  // Meaning: true if covariance was updated on this call
  bool covariance_updated{false};

  // Units: Tesla
  // Meaning: mean vector for the current window
  Eigen::Vector3d mean_t = Eigen::Vector3d::Zero();

  // Units: Tesla
  // Meaning: per-axis standard deviation for the current window
  Eigen::Vector3d stddev_t = Eigen::Vector3d::Zero();

  // Units: Tesla
  // Meaning: standard deviation of field magnitude
  double magnitude_stddev_t{0.0};

  // Units: Tesla^2
  // Meaning: current covariance estimate
  Eigen::Matrix3d covariance_t2 = Eigen::Matrix3d::Identity();
};

class CovarianceEstimator3D
{
public:
  bool Initialize(const CovarianceEstimatorConfig& config);
  void Reset();

  CovarianceUpdate Update(const Eigen::Vector3d& sample_t);

  const Eigen::Matrix3d& GetCovariance() const;

private:
  CovarianceUpdate ComputeUpdate();
  bool IsStationary(const CovarianceUpdate& update) const;

  CovarianceEstimatorConfig m_config;
  std::deque<Eigen::Vector3d> m_samples;
  Eigen::Matrix3d m_covariance = Eigen::Matrix3d::Identity();
  bool m_hasCovariance{false};
};

} // namespace Magnetometer
} // namespace OASIS
