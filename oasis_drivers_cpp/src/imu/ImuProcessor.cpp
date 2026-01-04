/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/ImuProcessor.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace OASIS::IMU
{
namespace
{
// Units: seconds
constexpr double kMinDtS = 1e-4;

// Units: seconds
constexpr double kMaxDtS = 1.0;

// Units: seconds
// Meaning: minimum EWMA time constant for noise estimation
constexpr double kMinTauS = 1e-3;

// Units: unitless
// Meaning: minimum EWMA update fraction to keep filter responsive
constexpr double kMinAlpha = 1e-6;

// Units: counts^2
// Meaning: diagonal jitter added to keep covariance invertible
constexpr double kDiagJitter = 1e-18;

// Units: nanoseconds per second
constexpr double kNsPerSecond = 1e9;

Mat3 ScaleMat3(const Mat3& m, double scale)
{
  Mat3 out{};
  for (std::size_t i = 0; i < 3; ++i)
  {
    for (std::size_t j = 0; j < 3; ++j)
      out[i][j] = m[i][j] * scale;
  }
  return out;
}

Vec3 ScaleCounts(const Vec3& counts, double scale)
{
  return {counts[0] * scale, counts[1] * scale, counts[2] * scale};
}

Vec3 Subtract(const Vec3& a, const Vec3& b)
{
  return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

Vec3 AddScaled(const Vec3& a, const Vec3& b, double scale)
{
  return {a[0] + b[0] * scale, a[1] + b[1] * scale, a[2] + b[2] * scale};
}

Mat3 Outer(const Vec3& a, const Vec3& b)
{
  Mat3 out{};
  for (std::size_t i = 0; i < 3; ++i)
  {
    for (std::size_t j = 0; j < 3; ++j)
      out[i][j] = a[i] * b[j];
  }
  return out;
}

Mat3 AddScaledMat3(const Mat3& a, const Mat3& b, double scale)
{
  Mat3 out{};
  for (std::size_t i = 0; i < 3; ++i)
  {
    for (std::size_t j = 0; j < 3; ++j)
      out[i][j] = a[i][j] + b[i][j] * scale;
  }
  return out;
}

Mat3 SymmetrizeMat3(const Mat3& m)
{
  Mat3 out{};
  for (std::size_t i = 0; i < 3; ++i)
  {
    for (std::size_t j = 0; j < 3; ++j)
      out[i][j] = 0.5 * (m[i][j] + m[j][i]);
  }
  return out;
}

Mat3 ZeroMat3()
{
  return {};
}

Vec3 Multiply(const Mat3& a, const Vec3& v)
{
  Vec3 out{0.0, 0.0, 0.0};
  for (std::size_t i = 0; i < 3; ++i)
  {
    for (std::size_t j = 0; j < 3; ++j)
      out[i] += a[i][j] * v[j];
  }
  return out;
}

double ClampDt(double dt_s)
{
  return std::clamp(dt_s, kMinDtS, kMaxDtS);
}
} // namespace

void ImuProcessor::RunningStats::Reset()
{
  count = 0;
  mean = 0.0;
  m2 = 0.0;
}

void ImuProcessor::RunningStats::Update(double value)
{
  ++count;

  const double delta = value - mean;
  mean += delta / static_cast<double>(count);

  const double delta2 = value - mean;
  m2 += delta * delta2;
}

double ImuProcessor::RunningStats::Variance() const
{
  if (count < 2)
    return 0.0;

  const double denom = static_cast<double>(count - 1);
  return m2 / denom;
}

void ImuProcessor::EwmaCovariance3::Configure(double tau_s_in, double min_variance_floor_counts2)
{
  tau_s = std::max(tau_s_in, kMinTauS);
  min_variance_floor = std::max(min_variance_floor_counts2, 0.0);
}

void ImuProcessor::EwmaCovariance3::Reset()
{
  initialized = false;
  mean = {0.0, 0.0, 0.0};
  cov = ZeroMat3();
}

void ImuProcessor::EwmaCovariance3::Update(const Vec3& sample, double dt_s)
{
  const double dt = ClampDt(dt_s);
  const double tau = std::max(tau_s, kMinTauS);

  double alpha = 1.0 - std::exp(-dt / tau);
  alpha = std::clamp(alpha, kMinAlpha, 1.0);

  if (!initialized)
  {
    mean = sample;
    cov = ZeroMat3();
    for (std::size_t i = 0; i < 3; ++i)
    {
      cov[i][i] = std::max(cov[i][i], min_variance_floor);
      cov[i][i] += kDiagJitter;
    }
    initialized = true;
    return;
  }

  const Vec3 mean_new = AddScaled(mean, Subtract(sample, mean), alpha);
  const Vec3 delta = Subtract(sample, mean_new);
  const Mat3 outer_dd = Outer(delta, delta);

  cov = AddScaledMat3(ScaleMat3(cov, 1.0 - alpha), outer_dd, alpha);
  cov = SymmetrizeMat3(cov);

  for (std::size_t i = 0; i < 3; ++i)
  {
    cov[i][i] = std::max(cov[i][i], min_variance_floor);
    cov[i][i] += kDiagJitter;
  }

  mean = mean_new;
}

Mat3 ImuProcessor::EwmaCovariance3::Covariance() const
{
  return cov;
}

bool ImuProcessor::Initialize(const Config& cfg)
{
  if (cfg.gravity_mps2 <= 0.0)
    return false;

  if (cfg.accel_scale_mps2_per_count <= 0.0)
    return false;

  if (cfg.gyro_scale_rads_per_count <= 0.0)
    return false;

  m_cfg = cfg;

  m_temperature.Reset();
  m_stationaryDetector.Reset();
  m_solver.Reset();
  m_gyroBiasEstimator.Reset();
  m_temperatureStats.Reset();
  m_accelNoise.Configure(m_cfg.noise_tau_s, m_cfg.noise_floor_accel_counts2);
  m_gyroNoise.Configure(m_cfg.noise_tau_s, m_cfg.noise_floor_gyro_counts2);
  m_accelNoise.Reset();
  m_gyroNoise.Reset();

  m_calibrationReady = false;
  m_calibrationRecord = {};
  m_hasCalibrationRecord = false;

  ImuCalibrationFile file;
  ImuCalibrationRecord record;
  if (file.Load(m_cfg.calibration_path, record) && record.calib.valid)
  {
    m_mode = Mode::Driver;
    m_calibrationRecord = record;
    m_hasCalibrationRecord = true;
  }
  else
  {
    m_mode = Mode::Calibration;
  }

  return true;
}

ImuProcessor::Output ImuProcessor::Update(int16_t ax,
                                          int16_t ay,
                                          int16_t az,
                                          int16_t gx,
                                          int16_t gy,
                                          int16_t gz,
                                          int16_t tempRaw,
                                          double dt_s,
                                          double stamp_s)
{
  Output out{};
  out.mode = m_mode;

  const double dt_clamped = ClampDt(dt_s);

  const Vec3 accel_counts{static_cast<double>(ax), static_cast<double>(ay),
                          static_cast<double>(az)};
  const Vec3 gyro_counts{static_cast<double>(gx), static_cast<double>(gy), static_cast<double>(gz)};

  const Vec3 accel_mps2 = ScaleCounts(accel_counts, m_cfg.accel_scale_mps2_per_count);
  const Vec3 gyro_rads = ScaleCounts(gyro_counts, m_cfg.gyro_scale_rads_per_count);

  const ImuTemperature::Sample temp_sample = m_temperature.ProcessRaw(tempRaw, dt_clamped);

  ImuSample raw_sample{};
  raw_sample.accel_mps2 = accel_mps2;
  raw_sample.gyro_rads = gyro_rads;
  raw_sample.temperature_c = temp_sample.temperatureC;
  raw_sample.temperature_var_c2 = temp_sample.varianceC2;
  raw_sample.stamp_s = stamp_s;

  if (m_mode == Mode::Driver)
  {
    raw_sample.accel_cov_mps2_2 = m_calibrationRecord.measurement_noise.accel_cov_mps2_2;
    raw_sample.gyro_cov_rads2_2 = m_calibrationRecord.measurement_noise.gyro_cov_rads2_2;

    out.raw = raw_sample;
    out.has_raw = true;

    ImuSample corrected_sample = raw_sample;
    const Vec3 accel_unbiased = Subtract(accel_mps2, m_calibrationRecord.calib.accel_bias_mps2);
    corrected_sample.accel_mps2 = Multiply(m_calibrationRecord.calib.accel_A, accel_unbiased);
    corrected_sample.gyro_rads = Subtract(gyro_rads, m_calibrationRecord.calib.gyro_bias_rads);

    out.corrected = corrected_sample;
    out.has_corrected = true;

    return out;
  }

  m_temperatureStats.Update(temp_sample.temperatureC);
  m_accelNoise.Update(accel_counts, dt_clamped);
  m_gyroNoise.Update(gyro_counts, dt_clamped);

  const Mat3 accel_cov_counts2 = m_accelNoise.Covariance();
  const Mat3 gyro_cov_counts2 = m_gyroNoise.Covariance();

  const double accel_scale2 = m_cfg.accel_scale_mps2_per_count * m_cfg.accel_scale_mps2_per_count;
  const double gyro_scale2 = m_cfg.gyro_scale_rads_per_count * m_cfg.gyro_scale_rads_per_count;

  raw_sample.accel_cov_mps2_2 = ScaleMat3(accel_cov_counts2, accel_scale2);
  raw_sample.gyro_cov_rads2_2 = ScaleMat3(gyro_cov_counts2, gyro_scale2);

  out.raw = raw_sample;
  out.has_raw = true;

  StationaryDetector::Noise noise{};
  noise.accel_cov_mps2_2 = raw_sample.accel_cov_mps2_2;
  noise.gyro_cov_rads2_2 = raw_sample.gyro_cov_rads2_2;

  const StationaryDetector::Status status = m_stationaryDetector.Update(raw_sample, noise);
  if (status.stationary)
  {
    m_gyroBiasEstimator.Update(status.mean_gyro_rads, status.cov_gyro_rads2_2, status.window_count);
    m_solver.AddSample(raw_sample.accel_mps2);

    if (!m_calibrationReady)
    {
      AccelCalibrationSolver::Result result{};
      if (m_solver.Solve(m_cfg.gravity_mps2, result))
      {
        ImuCalibrationRecord record{};

        const double stamp_ns = std::max(0.0, stamp_s * kNsPerSecond);
        record.created_unix_ns = static_cast<std::uint64_t>(stamp_ns);
        record.gravity_mps2 = m_cfg.gravity_mps2;
        record.fit_sample_count = result.sample_count;
        record.measurement_noise.accel_cov_mps2_2 = raw_sample.accel_cov_mps2_2;
        record.measurement_noise.gyro_cov_rads2_2 = raw_sample.gyro_cov_rads2_2;
        record.accel_ellipsoid = result.ellipsoid;

        record.calib.accel_bias_mps2 = result.accel_bias_mps2;
        record.calib.accel_A = result.accel_A;
        record.calib.accel_param_cov = result.accel_param_cov;
        record.calib.rms_residual_mps2 = result.rms_residual_mps2;
        if (m_gyroBiasEstimator.IsInitialized())
        {
          record.calib.gyro_bias_rads = m_gyroBiasEstimator.GetBias();
          record.calib.gyro_bias_cov_rads2_2 = m_gyroBiasEstimator.GetCov();
        }
        else
        {
          const double window_count = std::max(1.0, static_cast<double>(status.window_count));
          record.calib.gyro_bias_rads = status.mean_gyro_rads;
          record.calib.gyro_bias_cov_rads2_2 =
              ScaleMat3(status.cov_gyro_rads2_2, 1.0 / window_count);
        }
        record.calib.temperature_c = m_temperatureStats.mean;
        record.calib.temperature_var_c2 = m_temperatureStats.Variance();
        record.calib.valid = true;

        m_calibrationRecord = record;
        m_hasCalibrationRecord = true;
        m_calibrationReady = true;
      }
    }
  }

  out.calibration_ready = m_calibrationReady;
  if (m_calibrationReady)
    out.calibration_record = m_calibrationRecord;

  return out;
}

ImuProcessor::Mode ImuProcessor::GetMode() const
{
  return m_mode;
}

bool ImuProcessor::HasCalibrationRecord() const
{
  return m_hasCalibrationRecord;
}

const ImuCalibrationRecord& ImuProcessor::GetCalibrationRecord() const
{
  return m_calibrationRecord;
}

void ImuProcessor::Reset()
{
  m_temperature.Reset();

  if (m_mode == Mode::Calibration)
  {
    ResetCalibrationState();
    m_calibrationRecord = {};
    m_hasCalibrationRecord = false;
  }

  m_calibrationReady = false;
}

void ImuProcessor::ResetCalibrationState()
{
  m_stationaryDetector.Reset();
  m_solver.Reset();
  m_gyroBiasEstimator.Reset();
  m_temperatureStats.Reset();
  m_accelNoise.Reset();
  m_gyroNoise.Reset();
}
} // namespace OASIS::IMU
