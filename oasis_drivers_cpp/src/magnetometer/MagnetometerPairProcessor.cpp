/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "magnetometer/MagnetometerPairProcessor.h"

#include <cmath>
#include <filesystem>
#include <utility>

namespace OASIS
{
namespace Magnetometer
{
namespace
{
constexpr double kMilliGaussToTesla = 1e-7;
} // namespace

MagnetometerPairProcessor::MagnetometerPairProcessor(
    IMagnetometerPairSampler& sampler, MagnetometerSamplingConfig sampling_config,
    MagnetometerProcessingConfig processing_config,
    MagnetometerCalibrationConfig calibration_config)
  : m_sampler(sampler),
    m_samplingConfig(std::move(sampling_config)),
    m_processingConfig(std::move(processing_config)),
    m_calibrationConfig(std::move(calibration_config))
{
}

bool MagnetometerPairProcessor::Initialize()
{
  CovarianceEstimatorConfig covarianceConfig;
  covarianceConfig.window_size = m_processingConfig.stationary_window_size;
  covarianceConfig.prior_strength_samples = m_processingConfig.prior_strength_samples;
  covarianceConfig.ewma_tau_sec = m_processingConfig.ewma_tau_sec;
  covarianceConfig.dt_sec = m_samplingConfig.publish_period_sec;

  covarianceConfig.gate.axis_std_threshold_t =
      m_processingConfig.stationary_axis_std_threshold_t;
  covarianceConfig.gate.magnitude_std_threshold_t =
      m_processingConfig.stationary_magnitude_std_threshold_t;

  covarianceConfig.prior_covariance = ComputePriorCovariance();

  return m_covarianceEstimator.Initialize(covarianceConfig);
}

bool MagnetometerPairProcessor::Tick(MagnetometerTickOutput& output)
{
  output = MagnetometerTickOutput{};

  Eigen::Vector3d setSample = Eigen::Vector3d::Zero();
  Eigen::Vector3d resetSample = Eigen::Vector3d::Zero();
  if (!m_sampler.SamplePair(setSample, resetSample))
  {
    output.warning = m_sampler.GetLastError();
    return false;
  }

  output.field_t = (setSample - resetSample) * 0.5;
  output.offset_t = (setSample + resetSample) * 0.5;

  const CovarianceUpdate update = m_covarianceEstimator.Update(output.field_t);
  output.is_stationary = update.is_stationary;
  output.covariance_updated = update.covariance_updated;
  output.covariance_t2 = update.covariance_t2;

  const auto now = std::chrono::steady_clock::now();

  if (m_processingConfig.offset_spike_threshold_t > 0.0 &&
      m_processingConfig.offset_spike_inflation_factor > 1.0)
  {
    if (output.offset_t.norm() >= m_processingConfig.offset_spike_threshold_t)
    {
      output.offset_spike_triggered = true;
      m_offsetSpikeInflationUntil =
          now + std::chrono::duration<double>(m_processingConfig.offset_spike_inflation_sec);
    }

    if (m_offsetSpikeInflationUntil && now < *m_offsetSpikeInflationUntil)
    {
      output.offset_spike_inflated = true;
      output.covariance_t2 *= m_processingConfig.offset_spike_inflation_factor;
    }
  }

  output.covariance_finite = output.covariance_t2.allFinite();

  if (m_calibrationConfig.continuous_calibration)
  {
    MaybeWriteCalibration(output.offset_t, output.covariance_t2,
                          output.covariance_updated, output);
  }

  return true;
}

Eigen::Matrix3d MagnetometerPairProcessor::ComputePriorCovariance() const
{
  const double datasheetNoiseT = m_processingConfig.datasheet_rms_noise_mg *
                                 kMilliGaussToTesla;
  const double repeatT = m_processingConfig.set_reset_repeatability_mg *
                         kMilliGaussToTesla;

  const double sigmaRawT = std::sqrt((datasheetNoiseT * datasheetNoiseT) +
                                     (repeatT * repeatT));
  const double sigmaOutT = sigmaRawT / std::sqrt(2.0);
  const double variance = sigmaOutT * sigmaOutT;

  return Eigen::Matrix3d::Identity() * variance;
}

bool MagnetometerPairProcessor::MaybeWriteCalibration(
    const Eigen::Vector3d& offset_t, const Eigen::Matrix3d& covariance_t2,
    bool covariance_updated, MagnetometerTickOutput& output)
{
  if (!covariance_updated || m_calibrationConfig.calibration_path.empty())
    return false;

  const auto now = std::chrono::steady_clock::now();
  const auto systemNow = std::chrono::system_clock::now();

  if (m_lastCalibrationWrite)
  {
    const double elapsed =
        std::chrono::duration_cast<std::chrono::duration<double>>(now -
                                                                   *m_lastCalibrationWrite)
            .count();
    if (elapsed < m_calibrationConfig.write_cooldown_sec)
      return false;
  }

  std::error_code error;
  std::filesystem::create_directories(
      m_calibrationConfig.calibration_path.parent_path(), error);
  if (error)
  {
    output.warning =
        "Failed to create calibration directory: " + error.message();
    return false;
  }

  MagnetometerCalibrationRecord record;
  record.timestamp_s =
      std::chrono::duration_cast<std::chrono::duration<double>>(systemNow.time_since_epoch())
          .count();
  record.bandwidth_mode = m_calibrationConfig.bandwidth_mode;
  record.raw_rate_hz = m_calibrationConfig.raw_rate_hz;
  record.covariance_t2 = covariance_t2;
  record.offset_t = offset_t;

  const bool wrote = m_calibrationFile.Write(m_calibrationConfig.calibration_path, record);
  if (!wrote)
  {
    output.warning = "Failed to write magnetometer calibration";
    return false;
  }

  m_lastCalibrationWrite = now;
  output.calibration_written = true;
  return true;
}

} // namespace Magnetometer
} // namespace OASIS
