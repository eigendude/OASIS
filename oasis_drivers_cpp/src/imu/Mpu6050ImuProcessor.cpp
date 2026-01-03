/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/Mpu6050ImuProcessor.h"

#include <algorithm>
#include <array>
#include <filesystem>

using namespace OASIS::IMU;

namespace
{
// Fixed axis remap for the MPU6050 breakout: sensor Y is forward (imu X),
// sensor X is right (imu -Y), and sensor Z is up.
constexpr Mpu6050ImuProcessor::AxisRemap MPU6050_AXIS_REMAP{{1, 0, 2}, {1, -1, 1}};

std::array<double, 3> RemapAxes(const std::array<double, 3>& source,
                                const Mpu6050ImuProcessor::AxisRemap& remap)
{
  std::array<double, 3> remapped{0.0, 0.0, 0.0};

  for (size_t axis = 0; axis < remapped.size(); ++axis)
    remapped[axis] = static_cast<double>(remap.sign[axis]) * source[remap.map[axis]];

  return remapped;
}

std::array<double, 3> RemapVariances(const std::array<double, 3>& variances,
                                     const Mpu6050ImuProcessor::AxisRemap& remap)
{
  std::array<double, 3> remapped{0.0, 0.0, 0.0};

  for (size_t axis = 0; axis < remapped.size(); ++axis)
    remapped[axis] = variances[remap.map[axis]];

  return remapped;
}

AccelCalibrator::Mat3 RemapCovariance(const AccelCalibrator::Mat3& covariance,
                                      const Mpu6050ImuProcessor::AxisRemap& remap)
{
  AccelCalibrator::Mat3 remapped{};

  for (size_t row = 0; row < 3; ++row)
  {
    for (size_t col = 0; col < 3; ++col)
    {
      const int sign_row = remap.sign[row];
      const int sign_col = remap.sign[col];
      remapped[row][col] = static_cast<double>(sign_row * sign_col) *
                           covariance[remap.map[row]][remap.map[col]];
    }
  }

  return remapped;
}

} // namespace

void Mpu6050ImuProcessor::Reset()
{
  m_accelNoise.Reset();
  m_gyroNoise.Reset();
  m_gyroBiasEstimator.Reset();
  m_accelCalibrator.Reset();
  m_use_cached_noise = false;
  m_cached_accel_noise_stddev = {0.0, 0.0, 0.0};
  m_cached_gyro_noise_stddev = {0.0, 0.0, 0.0};
}

void Mpu6050ImuProcessor::ConfigureCalibration(const std::filesystem::path& cachePath,
                                               const std::string& frameId)
{
  m_calibrationCachePath = cachePath;
  m_calibrationFrameId = frameId;

  AccelCalibrator::Config config{};
  config.gravity_mps2 = m_gravity;
  m_accelCalibrator.Configure(config, m_calibrationCachePath, m_calibrationFrameId);
}

bool Mpu6050ImuProcessor::LoadCachedCalibration()
{
  const bool loaded = m_accelCalibrator.LoadCache();

  if (loaded)
  {
    const auto& calib = m_accelCalibrator.GetCalibration();
    if (m_accelCalibrator.HasCalibratedBaseline())
    {
      m_cached_accel_noise_stddev = calib.accel_noise_stddev_mps2;
      m_cached_gyro_noise_stddev = calib.gyro_noise_stddev_rads;
      m_use_cached_noise = true;
    }
    else if (m_accelCalibrator.HasRawBaseline())
    {
      m_cached_accel_noise_stddev = calib.raw_accel_noise_stddev_mps2;
      m_cached_gyro_noise_stddev = calib.raw_gyro_noise_stddev_rads;
      m_use_cached_noise = true;
    }
  }

  return loaded;
}

void Mpu6050ImuProcessor::SetCalibrationMode(bool enabled)
{
  m_accelCalibrator.SetCalibrationMode(enabled);
}

Mpu6050ImuProcessor::ProcessedOutputs Mpu6050ImuProcessor::ProcessRaw(int16_t ax,
                                                                      int16_t ay,
                                                                      int16_t az,
                                                                      int16_t gx,
                                                                      int16_t gy,
                                                                      int16_t gz,
                                                                      double dt_s,
                                                                      double temperature_c,
                                                                      double timestamp_s)
{
  ProcessedOutputs outputs{};

  const std::array<double, 3> accel_sensor_mps2{static_cast<double>(ax) * m_accelScale,
                                                static_cast<double>(ay) * m_accelScale,
                                                static_cast<double>(az) * m_accelScale};
  const std::array<double, 3> gyro_sensor_rads{static_cast<double>(gx) * m_gyroScale,
                                               static_cast<double>(gy) * m_gyroScale,
                                               static_cast<double>(gz) * m_gyroScale};

  const auto accel_sigma2_counts = m_accelNoise.Update(ax, ay, az, dt_s);
  const auto gyro_sigma2_counts = m_gyroNoise.Update(gx, gy, gz, dt_s);

  const double accel_floor = (m_accelScale * m_accelScale) / 12.0;
  const double gyro_floor = (m_gyroScale * m_gyroScale) / 12.0;

  std::array<double, 3> accel_var_sensor_mps2_2{0.0, 0.0, 0.0};
  std::array<double, 3> gyro_var_sensor_rads2_2{0.0, 0.0, 0.0};

  for (size_t axis = 0; axis < accel_sigma2_counts.size(); ++axis)
  {
    const double accel_variance = accel_sigma2_counts[axis] * (m_accelScale * m_accelScale);
    const double gyro_variance = gyro_sigma2_counts[axis] * (m_gyroScale * m_gyroScale);

    accel_var_sensor_mps2_2[axis] = std::max(accel_variance, accel_floor);
    gyro_var_sensor_rads2_2[axis] = std::max(gyro_variance, gyro_floor);
  }

  // Remap to link frame
  const std::array<double, 3> accel_body_mps2 = RemapAxes(accel_sensor_mps2, MPU6050_AXIS_REMAP);
  const std::array<double, 3> gyro_body_rads = RemapAxes(gyro_sensor_rads, MPU6050_AXIS_REMAP);
  const std::array<double, 3> gyro_var_body_rads2_2 =
      RemapVariances(gyro_var_sensor_rads2_2, MPU6050_AXIS_REMAP);

  AccelCalibrator::Mat3 accel_cov_sensor_mps2_2{};
  for (size_t axis = 0; axis < 3; ++axis)
    accel_cov_sensor_mps2_2[axis][axis] = accel_var_sensor_mps2_2[axis];

  const AccelCalibrator::Mat3 accel_cov_body_mps2_2 =
      RemapCovariance(accel_cov_sensor_mps2_2, MPU6050_AXIS_REMAP);

  // Record raw measurements
  outputs.imu_raw.accel_mps2 = accel_body_mps2;
  outputs.imu_raw.gyro_rads = gyro_body_rads;
  outputs.imu_raw.accel_cov_mps2_2 = accel_cov_body_mps2_2;
  outputs.imu_raw.gyro_var_rads2_2 = gyro_var_body_rads2_2;

  // Update gyro bias
  m_gyroBiasEstimator.Update(gyro_body_rads, accel_body_mps2, m_gravity);

  // Run calibration logic (stationary detection, ellipsoid fit, autosave)
  AccelCalibrator::Sample sample{};
  sample.accel_mps2 = accel_body_mps2;
  sample.gyro_rads = gyro_body_rads;
  sample.accel_cov_mps2_2 = accel_cov_body_mps2_2;
  sample.gyro_var_rads2_2 = gyro_var_body_rads2_2;
  sample.temperature_c = temperature_c;
  sample.timestamp_s = timestamp_s;
  outputs.calibration_status = m_accelCalibrator.Update(sample);

  if (!m_use_cached_noise)
  {
    if (m_accelCalibrator.HasCalibratedBaseline())
    {
      m_cached_accel_noise_stddev = m_accelCalibrator.GetCalibratedBaselineAccelNoiseStddev();
      m_cached_gyro_noise_stddev = m_accelCalibrator.GetCalibratedBaselineGyroNoiseStddev();
      m_use_cached_noise = true;
    }
    else if (m_accelCalibrator.HasRawBaseline())
    {
      m_cached_accel_noise_stddev = m_accelCalibrator.GetRawBaselineAccelNoiseStddev();
      m_cached_gyro_noise_stddev = m_accelCalibrator.GetRawBaselineGyroNoiseStddev();
      m_use_cached_noise = true;
    }
  }

  // Record calibrated measurements
  outputs.imu.accel_mps2 = m_accelCalibrator.ApplyAccel(accel_body_mps2);
  outputs.imu.gyro_rads = gyro_body_rads;
  outputs.imu.accel_cov_mps2_2 =
      m_accelCalibrator.ApplyAccelCovariance(accel_body_mps2, accel_cov_body_mps2_2);
  outputs.imu.gyro_var_rads2_2 = gyro_var_body_rads2_2;

  if (m_use_cached_noise)
  {
    std::array<double, 3> accel_var_cached{0.0, 0.0, 0.0};
    std::array<double, 3> gyro_var_cached{0.0, 0.0, 0.0};

    for (size_t axis = 0; axis < 3; ++axis)
    {
      accel_var_cached[axis] =
          m_cached_accel_noise_stddev[axis] * m_cached_accel_noise_stddev[axis];
      gyro_var_cached[axis] = m_cached_gyro_noise_stddev[axis] * m_cached_gyro_noise_stddev[axis];
    }

    AccelCalibrator::Mat3 accel_cov_cached{};
    for (size_t axis = 0; axis < 3; ++axis)
      accel_cov_cached[axis][axis] = accel_var_cached[axis];

    outputs.imu_raw.accel_cov_mps2_2 = accel_cov_cached;
    outputs.imu_raw.gyro_var_rads2_2 = gyro_var_cached;

    outputs.imu.accel_cov_mps2_2 =
        m_accelCalibrator.ApplyAccelCovariance(accel_body_mps2, accel_cov_cached);

    outputs.imu.gyro_var_rads2_2 = gyro_var_cached;
  }

  return outputs;
}
