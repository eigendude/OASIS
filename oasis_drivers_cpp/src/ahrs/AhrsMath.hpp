/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <optional>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace OASIS::AHRS
{
/*!
 * \brief Boot-time mounting calibration configuration
 */
struct AhrsMountingConfig
{
  /*!
   * \brief Stationary boot gravity window duration
   *
   * Units: seconds, expected range [0, +inf)
   */
  double calibration_duration_sec;

  /*!
   * \brief Maximum latest angular speed accepted during the boot solve
   *
   * Units: rad/s, expected range [0, +inf)
   */
  double stationary_angular_speed_threshold_rads;

  /*!
   * \brief Minimum accepted gravity samples before solving mounting
   *
   * Units: samples, expected range [1, +inf)
   */
  int min_sample_count;
};

/*!
 * \brief Solved fixed mounting from imu frame to parent frame
 */
struct AhrsMountingSolution
{
  /*!
   * \brief Fixed rotation mapping imu-frame vectors into parent frame
   *
   * Units: unit quaternion q_BI
   */
  Eigen::Quaterniond q_BI{Eigen::Quaterniond::Identity()};

  /*!
   * \brief Rotation matrix equivalent of q_BI
   *
   * Units: unitless direction cosine matrix
   */
  Eigen::Matrix3d R_BI{Eigen::Matrix3d::Identity()};

  /*!
   * \brief Solved mounting roll
   *
   * Units: radians
   */
  double roll_rad{0.0};

  /*!
   * \brief Solved mounting pitch
   *
   * Units: radians
   */
  double pitch_rad{0.0};

  /*!
   * \brief Accepted gravity samples used by the solve
   *
   * Units: samples
   */
  int sample_count{0};

  /*!
   * \brief Time covered by accepted boot gravity samples
   *
   * Units: seconds
   */
  double span_sec{0.0};
};

/*!
 * \brief Gravity consistency residual summary
 */
struct AhrsGravityResidual
{
  /*!
   * \brief Measured unit gravity direction after mounting
   *
   * Units: unit vector in parent frame
   */
  Eigen::Vector3d measured_direction{0.0, 0.0, -1.0};

  /*!
   * \brief Attitude-predicted unit gravity direction
   *
   * Units: unit vector in parent frame
   */
  Eigen::Vector3d predicted_direction{0.0, 0.0, -1.0};

  /*!
   * \brief Difference measured_direction - predicted_direction
   *
   * Units: unitless vector difference
   */
  Eigen::Vector3d residual_vector{0.0, 0.0, 0.0};

  /*!
   * \brief Euclidean norm of residual_vector
   *
   * Units: unitless
   */
  double residual_norm{0.0};

  /*!
   * \brief Covariance-aware residual distance when covariance is usable
   *
   * Units: standard deviations, NaN when unavailable
   */
  double mahalanobis_distance{std::numeric_limits<double>::quiet_NaN()};
};

/*!
 * \brief Boot-time gravity-only mounting solver
 */
class BootMountingCalibrator
{
public:
  explicit BootMountingCalibrator(AhrsMountingConfig config);

  std::optional<AhrsMountingSolution> AddGravitySample(
      int64_t timestamp_ns,
      const Eigen::Vector3d& gravity_imu,
      const std::optional<Eigen::Vector3d>& angular_velocity_rads);

private:
  AhrsMountingConfig m_config;
  std::optional<int64_t> m_start_ns;
  Eigen::Vector3d m_gravity_sum{0.0, 0.0, 0.0};
  int m_sample_count{0};
  std::optional<AhrsMountingSolution> m_solution;
};

bool IsFiniteVector3(const Eigen::Vector3d& vector);
bool IsFiniteQuaternion(const Eigen::Quaterniond& quaternion);
std::optional<Eigen::Quaterniond> NormalizeQuaternion(const Eigen::Quaterniond& quaternion);
Eigen::Quaterniond QuaternionFromRollPitchYaw(double roll_rad, double pitch_rad, double yaw_rad);
Eigen::Quaterniond QuaternionFromYaw(double yaw_rad);
double YawFromQuaternion(const Eigen::Quaterniond& quaternion);

/*!
 * \brief Compose IMU-to-world attitude with inverse IMU-to-base mounting
 *
 * \param q_WI Rotation mapping IMU-frame vectors into world coordinates
 * \param q_BI Rotation mapping IMU-frame vectors into base coordinates
 * \return Unit rotation q_WB mapping base-frame vectors into world coordinates
 */
Eigen::Quaterniond ComposeWorldFromBase(const Eigen::Quaterniond& q_WI,
                                        const Eigen::Quaterniond& q_BI);

/*!
 * \brief Express a base attitude relative to the odom/session frame
 *
 * \param q_OW Rotation mapping world-frame vectors into odom coordinates
 * \param q_WB Rotation mapping base-frame vectors into world coordinates
 * \return Unit rotation q_OB mapping base-frame vectors into odom coordinates
 */
Eigen::Quaterniond ComposeOdomFromBase(const Eigen::Quaterniond& q_OW,
                                       const Eigen::Quaterniond& q_WB);
Eigen::Matrix3d RotateCovariance(const Eigen::Matrix3d& rotation,
                                 const Eigen::Matrix3d& covariance);
std::array<double, 9> FlattenMatrix3(const Eigen::Matrix3d& matrix);
std::array<double, 36> EmbedLinearCovariance3(const Eigen::Matrix3d& matrix);
std::optional<Eigen::Matrix3d> ParseMatrix3(const std::array<double, 9>& values);
std::optional<Eigen::Matrix3d> ParseLinearCovariance3(const std::array<double, 36>& values);
std::optional<AhrsMountingSolution> SolveMountingFromGravity(
    const Eigen::Vector3d& mean_gravity_unit_imu, int sample_count, double span_sec);
std::optional<AhrsGravityResidual> ComputeGravityResidual(
    const Eigen::Vector3d& measured_gravity_base,
    const std::optional<Eigen::Matrix3d>& measured_gravity_covariance_base,
    const Eigen::Quaterniond& q_WB);
std::optional<Eigen::Matrix3d> GravityCovarianceToRollPitchCovariance(
    const Eigen::Vector3d& gravity_base,
    const std::optional<Eigen::Matrix3d>& gravity_covariance_base,
    double yaw_variance_rad2);
} // namespace OASIS::AHRS
