/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "magnetometer/CovarianceEstimator3D.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace OASIS
{
namespace Magnetometer
{

namespace
{
// Minimum diagonal epsilon to keep covariance PSD
// Units: Tesla^2
constexpr double kCovarianceEpsilonT2 = 1e-20;
} // namespace

bool CovarianceEstimator3D::Initialize(const CovarianceEstimatorConfig& config)
{
  if (config.window_size < 2 || config.dt_sec <= 0.0)
    return false;

  m_config = config;
  Reset();
  return true;
}

void CovarianceEstimator3D::Reset()
{
  m_samples.clear();
  m_covariance = m_config.prior_covariance;
  m_hasCovariance = false;
}

CovarianceUpdate CovarianceEstimator3D::Update(const Eigen::Vector3d& sample_t)
{
  m_samples.push_back(sample_t);
  if (m_samples.size() > m_config.window_size)
    m_samples.pop_front();

  if (m_samples.size() < m_config.window_size)
  {
    CovarianceUpdate update;
    update.covariance_t2 = m_covariance;
    return update;
  }

  CovarianceUpdate update = ComputeUpdate();
  update.is_stationary = IsStationary(update);

  if (update.is_stationary)
  {
    update.covariance_updated = true;

    const double windowSamples = static_cast<double>(m_config.window_size);
    const double denom = m_config.prior_strength_samples + (windowSamples - 1.0);
    const double priorWeight = m_config.prior_strength_samples / denom;
    const double sampleWeight = (windowSamples - 1.0) / denom;

    const Eigen::Matrix3d shrinkage =
        (priorWeight * m_config.prior_covariance) + (sampleWeight * update.covariance_t2);

    double alpha = 0.0;
    if (m_config.ewma_tau_sec > 0.0)
      alpha = std::exp(-m_config.dt_sec / m_config.ewma_tau_sec);
    if (!m_hasCovariance)
    {
      m_covariance = shrinkage;
      m_hasCovariance = true;
    }
    else
    {
      m_covariance = (alpha * m_covariance) + ((1.0 - alpha) * shrinkage);
    }

    m_covariance = 0.5 * (m_covariance + m_covariance.transpose());
    for (int i = 0; i < 3; ++i)
    {
      if (!std::isfinite(m_covariance(i, i)) || m_covariance(i, i) < kCovarianceEpsilonT2)
        m_covariance(i, i) = kCovarianceEpsilonT2;
    }

    update.covariance_t2 = m_covariance;
  }
  else
  {
    update.covariance_t2 = m_covariance;
  }

  return update;
}

const Eigen::Matrix3d& CovarianceEstimator3D::GetCovariance() const
{
  return m_covariance;
}

CovarianceUpdate CovarianceEstimator3D::ComputeUpdate()
{
  CovarianceUpdate update;

  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const auto& sample : m_samples)
    mean += sample;
  mean /= static_cast<double>(m_samples.size());

  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  std::vector<double> magnitudes;
  magnitudes.reserve(m_samples.size());

  for (const auto& sample : m_samples)
  {
    const Eigen::Vector3d delta = sample - mean;
    cov += delta * delta.transpose();
    magnitudes.push_back(sample.norm());
  }

  const double denom = static_cast<double>(m_samples.size() - 1);
  cov /= denom;

  double magnitudeMean = 0.0;
  for (double mag : magnitudes)
    magnitudeMean += mag;
  magnitudeMean /= static_cast<double>(magnitudes.size());

  double magnitudeVariance = 0.0;
  for (double mag : magnitudes)
  {
    const double delta = mag - magnitudeMean;
    magnitudeVariance += delta * delta;
  }
  magnitudeVariance /= denom;

  update.mean_t = mean;
  update.stddev_t = cov.diagonal().cwiseMax(0.0).cwiseSqrt();
  update.magnitude_stddev_t = std::sqrt(std::max(0.0, magnitudeVariance));
  update.covariance_t2 = cov;

  return update;
}

bool CovarianceEstimator3D::IsStationary(const CovarianceUpdate& update) const
{
  if (update.stddev_t.maxCoeff() > m_config.gate.axis_std_threshold_t)
    return false;

  if (update.magnitude_stddev_t > m_config.gate.magnitude_std_threshold_t)
    return false;

  return true;
}

} // namespace Magnetometer
} // namespace OASIS
