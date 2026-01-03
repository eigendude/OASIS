/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace OASIS::IMU
{
/**
 * Online accelerometer calibration for the MPU6050.
 *
 * This class remains ROS-agnostic and uses plain structs for inputs and
 * outputs. It performs online noise tracking, stationary detection, pose
 * clustering, ellipsoid fitting, and YAML cache management.
 */
class AccelCalibrator
{
public:
  /**
   * Configuration parameters that control calibration behavior.
   */
  struct Config
  {
    // Gravity magnitude in m/s^2.
    double gravity_mps2{9.80665};

    // Maximum window size used for stationary detection (samples).
    size_t window_size{10};

    // Variance multiplier for the CFAR-like stationary gate.
    double variance_threshold{3.0};

    // Mean multiplier for the gyro mean gate.
    double gyro_mean_threshold{3.0};

    // Merge tolerance for Mahalanobis clustering (sigma multiplier).
    double merge_sigma{2.0};

    // Maximum angular deviation (radians) accepted for axis coverage.
    double max_axis_angle_rad{0.436332}; // 25 degrees
  };

  /**
   * Raw IMU measurements provided to the calibrator.
   */
  struct Sample
  {
    // Linear acceleration in m/s^2 in the imu_link frame.
    std::array<double, 3> accel_mps2{0.0, 0.0, 0.0};

    // Per-axis accelerometer variance in (m/s^2)^2.
    std::array<double, 3> accel_var_mps2_2{0.0, 0.0, 0.0};

    // Angular velocity in rad/s in the imu_link frame.
    std::array<double, 3> gyro_rads{0.0, 0.0, 0.0};

    // Per-axis gyroscope variance in (rad/s)^2.
    std::array<double, 3> gyro_var_rads2_2{0.0, 0.0, 0.0};

    // Filtered temperature sample in Celsius.
    double temperature_c{0.0};

    // Monotonic timestamp in seconds used for autosave cooldowns.
    double timestamp_s{0.0};
  };

  /**
   * Calibration parameters used to correct accelerometer data.
   */
  struct Calibration
  {
    // Schema metadata
    int version{1};
    std::string frame_id{"imu_link"};
    double gravity_mps2{9.80665};
    std::uint64_t created_unix_ns{0};
    bool has_ellipsoid{true};

    // Noise statistics (Gaussian standard deviation per axis)
    std::array<double, 3> accel_noise_stddev_mps2{0.0, 0.0, 0.0};
    std::array<double, 3> gyro_noise_stddev_rads{0.0, 0.0, 0.0};

    // Accelerometer bias term b in a_cal = A * (a_raw - b)
    std::array<double, 3> bias_mps2{0.0, 0.0, 0.0};

    // Accelerometer correction matrix A (row-major 3x3)
    std::array<std::array<double, 3>, 3> A{{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}};

    // Parameter uncertainties (1 sigma) for bias (m/s^2)
    std::array<double, 3> bias_stddev_mps2{0.0, 0.0, 0.0};

    // Parameter uncertainties (1 sigma) for A
    std::array<std::array<double, 3>, 3> A_stddev{
        {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}};

    // Ellipsoid form for debugging: (a - center)^T W (a - center) = g^2
    std::array<std::array<double, 3>, 3> W{{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}};
    std::array<double, 3> center_mps2{0.0, 0.0, 0.0};

    // Fit quality metrics
    double rms_residual_mps2{0.0};
    size_t num_clusters{0};
    size_t num_samples{0};

    // Noise fit metadata
    size_t noise_stationary_samples{0};
    std::string noise_method;
    std::string noise_phase;

    // Raw baseline noise and bias estimates at rest
    std::array<double, 3> raw_accel_bias_mps2{0.0, 0.0, 0.0};
    std::array<double, 3> raw_gyro_bias_rads{0.0, 0.0, 0.0};
    std::array<double, 3> raw_accel_noise_stddev_mps2{0.0, 0.0, 0.0};
    std::array<double, 3> raw_gyro_noise_stddev_rads{0.0, 0.0, 0.0};
    size_t raw_stationary_samples{0};
    std::string raw_noise_method;

    // Calibrated baseline noise after ellipsoid fit
    std::array<double, 3> calibrated_noise_accel_stddev_mps2{0.0, 0.0, 0.0};
    std::array<double, 3> calibrated_noise_gyro_stddev_rads{0.0, 0.0, 0.0};
    size_t calibrated_stationary_samples{0};
    std::string calibrated_noise_method;

    // Stability of rest biases across stationary windows
    std::array<double, 3> gyro_bias_stddev_rads{0.0, 0.0, 0.0};
    std::array<double, 3> accel_bias_stddev_mps2{0.0, 0.0, 0.0};

    // Temperature summary of samples used for the fit
    double temperature_mean_c{0.0};
    double temperature_stddev_c{0.0};
  };

  /**
   * Stationary detector output and calibration status after an update.
   */
  struct UpdateStatus
  {
    // True when the latest window is classified as stationary.
    bool stationary{false};

    // True when enough coverage exists to solve the ellipsoid.
    bool has_axis_coverage{false};

    // Number of pose clusters accumulated for the fit.
    size_t num_clusters{0};

    // Total pose samples merged into clusters.
    size_t num_pose_samples{0};

    // True when a calibration matrix has been solved.
    bool has_solution{false};

    // True when the solution was written to disk during this update.
    bool wrote_cache{false};
  };

  struct WindowSample
  {
    // Mean of the acceleration window (m/s^2).
    std::array<double, 3> mean_accel{0.0, 0.0, 0.0};

    // Mean of the calibrated acceleration window (m/s^2).
    std::array<double, 3> mean_accel_cal{0.0, 0.0, 0.0};

    // Mean of the gyro window (rad/s).
    std::array<double, 3> mean_gyro{0.0, 0.0, 0.0};

    // Variance of the acceleration window (m/s^2)^2.
    std::array<double, 3> var_accel{0.0, 0.0, 0.0};

    // Variance of the calibrated acceleration window (m/s^2)^2.
    std::array<double, 3> var_accel_cal{0.0, 0.0, 0.0};

    // Variance of the gyro window (rad/s)^2.
    std::array<double, 3> var_gyro{0.0, 0.0, 0.0};

    // Mean and standard deviation of |a| over the window.
    double mean_norm{0.0};
    double stddev_norm{0.0};
  };

  AccelCalibrator() = default;

  void Configure(const Config& config,
                 const std::filesystem::path& cache_path,
                 const std::string& frame_id);

  void Reset();

  bool LoadCache();

  // Enable or disable online calibration when no cache is present.
  void SetCalibrationMode(bool enabled) { m_calibration_mode = enabled; }

  // Apply current calibration to an accel vector; returns input when invalid.
  std::array<double, 3> ApplyAccel(const std::array<double, 3>& accel_mps2) const;

  // Apply calibration to accelerometer variance (includes parameter uncertainty
  // when available).
  std::array<double, 3> ApplyAccelVariance(const std::array<double, 3>& accel_mps2,
                                           const std::array<double, 3>& accel_var_mps2_2) const;

  // Process one IMU sample. Calibration runs only when enabled.
  UpdateStatus Update(const Sample& sample);

  bool HasSolution() const { return m_calibration.has_value() && m_calibration->has_ellipsoid; }

  const Calibration& GetCalibration() const { return *m_calibration; }
  const std::array<double, 3>& GetAccelNoiseStddev() const { return m_noise_stddev_accel; }
  const std::array<double, 3>& GetGyroNoiseStddev() const { return m_noise_stddev_gyro; }
  bool HasRawBaseline() const { return m_raw_baseline_valid; }
  bool HasCalibratedBaseline() const { return m_calibrated_baseline_valid; }
  std::array<double, 3> GetRawBaselineAccelNoiseStddev() const;
  std::array<double, 3> GetRawBaselineGyroNoiseStddev() const;
  std::array<double, 3> GetCalibratedBaselineAccelNoiseStddev() const;
  std::array<double, 3> GetCalibratedBaselineGyroNoiseStddev() const;
  std::array<double, 3> GetRawBias() const { return m_raw_bias_accel; }
  std::array<double, 3> GetRawGyroBias() const { return m_raw_bias_gyro; }
  std::array<double, 3> GetBiasStabilityAccel() const;
  std::array<double, 3> GetBiasStabilityGyro() const;

private:
  struct Cluster
  {
    // Cluster mean in m/s^2.
    std::array<double, 3> mean{0.0, 0.0, 0.0};

    // Diagonal covariance estimate in (m/s^2)^2.
    std::array<double, 3> covariance{0.0, 0.0, 0.0};

    // Total weight (precision-weighted samples merged).
    double weight{0.0};

    // Number of pose measurements merged into this cluster.
    size_t count{0};

    // Online mean temperature in Celsius for this cluster.
    double temperature_mean_c{0.0};
  };

  void UpdateNoiseEstimates(const Sample& sample);
  void UpdateBaselineNoise(const WindowSample& stats);
  bool DetectStationary(const Sample& sample, const WindowSample& stats);
  void MergePose(const Sample& sample, const WindowSample& stats);
  bool HasAxisCoverage() const;
  double ComputeDirectionalSpread() const;
  bool FitEllipsoid();
  bool SaveCache(const Sample& sample);

  static Calibration MakeIdentityCalibration(double gravity_mps2, const std::string& frame_id);

  struct RunningStats
  {
    double mean{0.0};
    double m2{0.0};
    size_t count{0};

    void AddSample(double value)
    {
      ++count;
      const double delta = value - mean;
      mean += delta / static_cast<double>(count);
      const double delta2 = value - mean;
      m2 += delta * delta2;
    }

    double Variance() const
    {
      if (count < 2)
        return 0.0;
      return m2 / static_cast<double>(count - 1);
    }
  };

  Config m_config{};
  std::filesystem::path m_cache_path;
  std::string m_frame_id{"imu_link"};
  bool m_calibration_mode{false};

  // Noise tracking (stddev per axis)
  std::array<double, 3> m_noise_stddev_accel{0.0, 0.0, 0.0};
  std::array<double, 3> m_noise_stddev_gyro{0.0, 0.0, 0.0};
  bool m_noise_initialized{false};

  // Stationary window
  std::vector<WindowSample> m_window;

  // Pose clusters
  std::vector<Cluster> m_clusters;
  size_t m_total_pose_samples{0};

  // Fit status
  std::optional<Calibration> m_calibration;
  bool m_dirty{false};
  double m_last_save_time_s{0.0};

  // Slow gyro bias used to de-bias the mean gate for stationary detection.
  std::array<double, 3> m_gyro_bias_iir{0.0, 0.0, 0.0};
  bool m_gyro_bias_iir_init{false};

  // Baseline rest noise accumulation
  bool m_raw_baseline_valid{false};
  bool m_calibrated_baseline_valid{false};
  size_t m_raw_stationary_samples{0};
  size_t m_calibrated_stationary_samples{0};
  size_t m_consecutive_stationary{0};
  std::array<double, 3> m_raw_baseline_accel_var{0.0, 0.0, 0.0};
  std::array<double, 3> m_raw_baseline_gyro_var{0.0, 0.0, 0.0};
  std::array<double, 3> m_cal_baseline_accel_var{0.0, 0.0, 0.0};
  std::array<double, 3> m_cal_baseline_gyro_var{0.0, 0.0, 0.0};
  std::array<double, 3> m_raw_bias_accel{0.0, 0.0, 0.0};
  std::array<double, 3> m_raw_bias_gyro{0.0, 0.0, 0.0};
  std::array<RunningStats, 3> m_raw_bias_stats_accel;
  std::array<RunningStats, 3> m_raw_bias_stats_gyro;
};
} // namespace OASIS::IMU
