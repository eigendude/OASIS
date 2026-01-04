/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "magnetometer/CovarianceEstimator3D.h"
#include "magnetometer/IMagnetometerPairSampler.h"
#include "magnetometer/io/MagnetometerCalibrationFile.h"

#include <chrono>
#include <filesystem>
#include <optional>
#include <string>

#include <Eigen/Core>
#include <Eigen/Dense>

namespace OASIS
{
namespace Magnetometer
{

struct MagnetometerSamplingConfig
{
  // Units: seconds
  // Meaning: publish cadence used by the caller
  double publish_period_sec{0.0};
};

struct MagnetometerProcessingConfig
{
  // Units: samples
  // Meaning: number of samples in the stationary gate window
  std::size_t stationary_window_size{0};

  // Units: samples
  // Meaning: prior equivalent sample count for shrinkage
  double prior_strength_samples{0.0};

  // Units: seconds
  // Meaning: EWMA time constant for covariance smoothing
  double ewma_tau_sec{0.0};

  // Units: Tesla
  // Meaning: per-axis stddev threshold for stationary detection
  double stationary_axis_std_threshold_t{0.0};

  // Units: Tesla
  // Meaning: magnitude stddev threshold for stationary detection
  double stationary_magnitude_std_threshold_t{0.0};

  // Units: milli-Gauss RMS
  // Meaning: datasheet noise floor before pairing
  double datasheet_rms_noise_mg{0.0};

  // Units: milli-Gauss RMS
  // Meaning: repeatability between SET and RESET conversions
  double set_reset_repeatability_mg{0.0};

  // Units: Tesla
  // Meaning: offset magnitude that triggers covariance inflation
  double offset_spike_threshold_t{0.0};

  // Meaning: multiplicative inflation factor while an offset spike is active
  double offset_spike_inflation_factor{0.0};

  // Units: seconds
  // Meaning: duration to keep inflated covariance after spike detection
  double offset_spike_inflation_sec{0.0};
};

struct MagnetometerCalibrationConfig
{
  // Meaning: enable writing calibration records when covariance updates
  bool continuous_calibration{false};

  // Units: seconds
  // Meaning: cooldown between calibration file writes
  double write_cooldown_sec{0.0};

  // Meaning: destination path for calibration YAML output
  std::filesystem::path calibration_path;

  // Meaning: bandwidth mode setting to persist with calibration data
  std::uint8_t bandwidth_mode{0};

  // Units: Hz
  // Meaning: raw measurement rate to persist with calibration data
  std::uint16_t raw_rate_hz{0};
};

struct MagnetometerTickOutput
{
  // Units: Tesla
  // Meaning: paired magnetic field measurement (SET - RESET) / 2
  Eigen::Vector3d field_t = Eigen::Vector3d::Zero();

  // Units: Tesla
  // Meaning: paired offset estimate (SET + RESET) / 2
  Eigen::Vector3d offset_t = Eigen::Vector3d::Zero();

  // Units: Tesla^2
  // Meaning: covariance estimate for the magnetic field
  Eigen::Matrix3d covariance_t2 = Eigen::Matrix3d::Identity();

  // Meaning: true if stationary gating accepted the current window
  bool is_stationary{false};

  // Meaning: true if covariance was updated on this tick
  bool covariance_updated{false};

  // Meaning: true if covariance matrix contains only finite entries
  bool covariance_finite{false};

  // Meaning: true when an offset spike is detected for this tick
  bool offset_spike_triggered{false};

  // Meaning: true while covariance inflation is active
  bool offset_spike_inflated{false};

  // Meaning: true if calibration was written on this tick
  bool calibration_written{false};

  // Meaning: optional diagnostics describing why Tick() failed
  std::string warning;
};

// Meaning: generic processor for paired SET/RESET magnetometer sampling
// Architecture: sampler fetches raw SET/RESET pairs, processor performs
// pairing, gating, covariance estimation, and optional calibration writes
class MagnetometerPairProcessor
{
public:
  MagnetometerPairProcessor(IMagnetometerPairSampler& sampler,
                            MagnetometerSamplingConfig sampling_config,
                            MagnetometerProcessingConfig processing_config,
                            MagnetometerCalibrationConfig calibration_config);

  bool Initialize();

  bool Tick(MagnetometerTickOutput& output);

private:
  Eigen::Matrix3d ComputePriorCovariance() const;
  bool MaybeWriteCalibration(const Eigen::Vector3d& offset_t,
                             const Eigen::Matrix3d& covariance_t2,
                             bool covariance_updated,
                             MagnetometerTickOutput& output);

  IMagnetometerPairSampler& m_sampler;
  MagnetometerSamplingConfig m_samplingConfig;
  MagnetometerProcessingConfig m_processingConfig;
  MagnetometerCalibrationConfig m_calibrationConfig;

  CovarianceEstimator3D m_covarianceEstimator;
  MagnetometerCalibrationFile m_calibrationFile;

  std::optional<std::chrono::steady_clock::time_point> m_offsetSpikeInflationUntil;
  std::optional<std::chrono::steady_clock::time_point> m_lastCalibrationWrite;
};

} // namespace Magnetometer
} // namespace OASIS
