/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "imu/ImuTemperature.h"
#include "imu/ImuTypes.h"
#include "imu/StationaryDetector.h"
#include "imu/cal/AccelCalibrationSolver.h"
#include "imu/cal/GyroBiasEstimator.h"
#include "imu/io/ImuCalibrationFile.h"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <vector>

namespace OASIS::IMU
{
class ImuProcessor
{
public:
  enum class Mode
  {
    Driver,
    Calibration
  };

  struct Config
  {
    /*!
     * \brief Gravity magnitude used for calibration
     *
     * Units: m/s^2
     */
    double gravity_mps2{9.80665};

    /*!
     * \brief Accel scale conversion factor
     *
     * Units: (m/s^2)/count
     */
    double accel_scale_mps2_per_count{0.0};

    /*!
     * \brief Gyro scale conversion factor
     *
     * Units: (rad/s)/count
     */
    double gyro_scale_rads_per_count{0.0};

    /*!
     * \brief EWMA time constant for measurement-noise covariance estimation
     *
     * Units: seconds
     */
    double noise_tau_s{2.0};

    /*!
     * \brief Minimum accel variance floor for measurement-noise covariance
     *
     * Units: counts^2
     */
    double noise_floor_accel_counts2{1.0};

    /*!
     * \brief Minimum gyro variance floor for measurement-noise covariance
     *
     * Units: counts^2
     */
    double noise_floor_gyro_counts2{1.0};

    /*!
     * \brief Calibration file path to load
     *
     * Units: path
     */
    std::filesystem::path calibration_path;

    /*!
     * \brief Maximum angle between gravity directions to merge poses
     *
     * Units: degrees
     */
    double pose_cluster_angle_deg{15.0};

    /*!
     * \brief Minimum time between accepted stationary poses
     *
     * Units: seconds
     */
    double pose_min_interval_s{0.5};

    /*!
     * \brief Minimum number of pose clusters for coverage
     *
     * Units: count
     */
    std::size_t min_pose_clusters{6};

    /*!
     * \brief Axis coverage cosine threshold for pose clusters
     *
     * Units: unitless
     */
    double coverage_axis_cos{0.7};
  };

  struct Output
  {
    /*!
     * \brief True when the raw sample fields are populated
     */
    bool has_raw{false};

    /*!
     * \brief True when the corrected sample fields are populated
     */
    bool has_corrected{false};

    /*!
     * \brief Raw sample in IMU frame
     */
    ImuSample raw{};

    /*!
     * \brief Corrected sample in IMU frame
     */
    ImuSample corrected{};

    /*!
     * \brief Current processor mode
     */
    Mode mode{Mode::Calibration};

    /*!
     * \brief True when a calibration record is ready in calibration mode
     */
    bool calibration_ready{false};

    /*!
     * \brief Latest calibration record when ready
     */
    ImuCalibrationRecord calibration_record{};
  };

  bool Initialize(const Config& cfg);

  Output Update(int16_t ax,
                int16_t ay,
                int16_t az,
                int16_t gx,
                int16_t gy,
                int16_t gz,
                int16_t tempRaw,
                double dt_s,
                double stamp_s);

  Mode GetMode() const;
  bool HasCalibrationRecord() const;
  const ImuCalibrationRecord& GetCalibrationRecord() const;
  void Reset();

private:
  struct RunningStats
  {
    void Reset();
    void Update(double value);
    double Variance() const;

    std::size_t count{0};
    double mean{0.0};
    double m2{0.0};
  };

  struct EwmaCovariance3
  {
    void Configure(double tau_s, double min_variance_floor_counts2);
    void Reset();
    void Update(const Vec3& sample, double dt_s);
    Mat3 Covariance() const;

    bool initialized{false};

    // Units: seconds
    // Meaning: EWMA time constant for noise covariance adaptation
    double tau_s{2.0};
    double min_variance_floor{1.0};

    Vec3 mean{0.0, 0.0, 0.0};
    Mat3 cov{};
  };

  struct PoseCluster
  {
    /*!
     * \brief Unit gravity direction in the IMU frame
     *
     * Units: unitless
     */
    Vec3 dir_unit{0.0, 0.0, 0.0};

    /*!
     * \brief Running mean accel vector for this pose
     *
     * Units: m/s^2
     */
    Vec3 mean_accel_mps2{0.0, 0.0, 0.0};

    /*!
     * \brief Number of windows fused into this pose
     *
     * Units: count
     */
    std::size_t count{0};
  };

  static double Dot(const Vec3& a, const Vec3& b);
  static double Norm(const Vec3& a);
  static Vec3 NormalizeOrZero(const Vec3& v);

  std::size_t FindOrCreatePoseCluster(const Vec3& dir_unit, bool& created);
  void UpdatePoseCluster(std::size_t idx, const Vec3& mean_accel_mps2);

  bool HasSufficientCoverage() const;
  bool IsCalibrationImproved(const AccelCalibrationSolver::Result& result) const;

  void ResetCalibrationState();

  Config m_cfg{};
  Mode m_mode{Mode::Calibration};

  bool m_hasCalibrationRecord{false};
  bool m_calibrationReady{false};
  bool m_calibrationUpdated{false};

  ImuCalibrationRecord m_calibrationRecord{};

  ImuTemperature m_temperature;
  StationaryDetector m_stationaryDetector;
  AccelCalibrationSolver m_solver;
  GyroBiasEstimator m_gyroBiasEstimator;

  std::vector<PoseCluster> m_poseClusters;

  bool m_prevStationary{false};
  double m_lastPoseAcceptStampS{0.0};

  bool m_hasBestAccelFit{false};
  double m_bestAccelRmsMps2{0.0};
  std::uint32_t m_bestAccelSampleCount{0};

  RunningStats m_temperatureStats;
  EwmaCovariance3 m_accelNoise;
  EwmaCovariance3 m_gyroNoise;
};
} // namespace OASIS::IMU
