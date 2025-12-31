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

    // True when a calibration matrix has been solved.
    bool has_solution{false};

    // True when the solution was written to disk during this update.
    bool wrote_cache{false};
  };

  struct WindowSample
  {
    // Mean of the acceleration window (m/s^2).
    std::array<double, 3> mean_accel{0.0, 0.0, 0.0};

    // Mean of the gyro window (rad/s).
    std::array<double, 3> mean_gyro{0.0, 0.0, 0.0};

    // Variance of the acceleration window (m/s^2)^2.
    std::array<double, 3> var_accel{0.0, 0.0, 0.0};

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
  std::array<double, 3> Apply(const std::array<double, 3>& accel_mps2) const;

  // Process one IMU sample. Calibration runs only when enabled.
  UpdateStatus Update(const Sample& sample);

  bool HasSolution() const { return m_calibration.has_value(); }

  const Calibration& GetCalibration() const { return *m_calibration; }

  const std::array<double, 3>& GetAccelNoiseStddev() const { return m_noise_stddev_accel; }

  const std::array<double, 3>& GetGyroNoiseStddev() const { return m_noise_stddev_gyro; }

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
  bool DetectStationary(const Sample& sample, const WindowSample& stats) const;
  void MergePose(const Sample& sample, const WindowSample& stats);
  bool HasAxisCoverage() const;
  bool FitEllipsoid();
  bool SaveCache(const Sample& sample);

  static Calibration MakeIdentityCalibration(double gravity_mps2, const std::string& frame_id);

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
};
} // namespace OASIS::IMU
