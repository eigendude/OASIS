/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "ahrs/AhrsMath.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

using namespace OASIS::AHRS;

namespace
{
constexpr double kMinGravityNorm = 1.0e-9;
constexpr double kMinAttitudeCovarianceGravityNorm = 1.0e-3;
constexpr double kMinRollPitchObservableNorm = 1.0e-3;
constexpr double kNormalizedCovarianceFloor = 1.0e-9;
constexpr double kMinCovarianceDeterminant = 1.0e-30;

bool IsFiniteMatrix3(const Eigen::Matrix3d& matrix)
{
  return matrix.allFinite();
}

std::optional<Eigen::Vector3d> NormalizeVector(const Eigen::Vector3d& vector, double min_norm)
{
  if (!IsFiniteVector3(vector))
    return std::nullopt;

  const double norm = vector.norm();
  if (norm <= min_norm)
    return std::nullopt;

  return vector / norm;
}

std::optional<Eigen::Matrix3d> InvertMatrix3(const Eigen::Matrix3d& matrix)
{
  const double determinant = matrix.determinant();
  if (!std::isfinite(determinant) || std::abs(determinant) <= kMinCovarianceDeterminant)
    return std::nullopt;

  const Eigen::Matrix3d inverse = matrix.inverse();
  if (!IsFiniteMatrix3(inverse))
    return std::nullopt;

  return inverse;
}
} // namespace

BootMountingCalibrator::BootMountingCalibrator(AhrsMountingConfig config)
  : m_config(std::move(config))
{
  m_config.calibration_duration_sec = std::max(m_config.calibration_duration_sec, 0.0);
  m_config.stationary_angular_speed_threshold_rads =
      std::max(m_config.stationary_angular_speed_threshold_rads, 0.0);
  m_config.min_sample_count = std::max(m_config.min_sample_count, 1);
}

std::optional<AhrsMountingSolution> BootMountingCalibrator::AddGravitySample(
    int64_t timestamp_ns,
    const Eigen::Vector3d& gravity_imu,
    const std::optional<Eigen::Vector3d>& angular_velocity_rads)
{
  if (m_solution.has_value())
    return m_solution;

  if (angular_velocity_rads.has_value() && IsFiniteVector3(*angular_velocity_rads) &&
      angular_velocity_rads->norm() > m_config.stationary_angular_speed_threshold_rads)
  {
    return std::nullopt;
  }

  const std::optional<Eigen::Vector3d> gravity_unit = NormalizeVector(gravity_imu, kMinGravityNorm);
  if (!gravity_unit.has_value())
    return std::nullopt;

  if (!m_start_ns.has_value())
    m_start_ns = timestamp_ns;

  m_gravity_sum += *gravity_unit;
  ++m_sample_count;

  const double span_sec = static_cast<double>(timestamp_ns - *m_start_ns) * 1.0e-9;
  if (span_sec < m_config.calibration_duration_sec)
    return std::nullopt;

  if (m_sample_count < m_config.min_sample_count)
    return std::nullopt;

  const std::optional<Eigen::Vector3d> mean_gravity =
      NormalizeVector(m_gravity_sum, kMinGravityNorm);
  if (!mean_gravity.has_value())
    return std::nullopt;

  m_solution = SolveMountingFromGravity(*mean_gravity, m_sample_count, span_sec);
  return m_solution;
}

bool OASIS::AHRS::IsFiniteVector3(const Eigen::Vector3d& vector)
{
  return vector.allFinite();
}

bool OASIS::AHRS::IsFiniteQuaternion(const Eigen::Quaterniond& quaternion)
{
  return std::isfinite(quaternion.x()) && std::isfinite(quaternion.y()) &&
         std::isfinite(quaternion.z()) && std::isfinite(quaternion.w());
}

std::optional<Eigen::Quaterniond> OASIS::AHRS::NormalizeQuaternion(
    const Eigen::Quaterniond& quaternion)
{
  if (!IsFiniteQuaternion(quaternion))
    return std::nullopt;

  const double norm = quaternion.norm();
  if (norm <= 0.0)
    return std::nullopt;

  Eigen::Quaterniond normalized(quaternion.w() / norm, quaternion.x() / norm, quaternion.y() / norm,
                                quaternion.z() / norm);
  return normalized;
}

Eigen::Quaterniond OASIS::AHRS::QuaternionFromRollPitchYaw(double roll_rad,
                                                           double pitch_rad,
                                                           double yaw_rad)
{
  const Eigen::AngleAxisd roll(roll_rad, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch(pitch_rad, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw(yaw_rad, Eigen::Vector3d::UnitZ());
  return Eigen::Quaterniond(yaw * pitch * roll).normalized();
}

Eigen::Quaterniond OASIS::AHRS::QuaternionFromYaw(double yaw_rad)
{
  return Eigen::Quaterniond(Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()));
}

double OASIS::AHRS::YawFromQuaternion(const Eigen::Quaterniond& quaternion)
{
  return std::atan2(2.0 * (quaternion.w() * quaternion.z() + quaternion.x() * quaternion.y()),
                    1.0 -
                        2.0 * (quaternion.y() * quaternion.y() + quaternion.z() * quaternion.z()));
}

Eigen::Quaterniond OASIS::AHRS::ComposeWorldFromBase(const Eigen::Quaterniond& q_WI,
                                                     const Eigen::Quaterniond& q_BI)
{
  const Eigen::Quaterniond q_IB = q_BI.conjugate();
  return (q_WI * q_IB).normalized();
}

Eigen::Quaterniond OASIS::AHRS::ComposeOdomFromBase(const Eigen::Quaterniond& q_OW,
                                                    const Eigen::Quaterniond& q_WB)
{
  return (q_OW * q_WB).normalized();
}

Eigen::Matrix3d OASIS::AHRS::RotateCovariance(const Eigen::Matrix3d& rotation,
                                              const Eigen::Matrix3d& covariance)
{
  return rotation * covariance * rotation.transpose();
}

std::array<double, 9> OASIS::AHRS::FlattenMatrix3(const Eigen::Matrix3d& matrix)
{
  return {matrix(0, 0), matrix(0, 1), matrix(0, 2), matrix(1, 0), matrix(1, 1),
          matrix(1, 2), matrix(2, 0), matrix(2, 1), matrix(2, 2)};
}

std::array<double, 36> OASIS::AHRS::EmbedLinearCovariance3(const Eigen::Matrix3d& matrix)
{
  std::array<double, 36> covariance{};
  covariance[0] = matrix(0, 0);
  covariance[1] = matrix(0, 1);
  covariance[2] = matrix(0, 2);
  covariance[6] = matrix(1, 0);
  covariance[7] = matrix(1, 1);
  covariance[8] = matrix(1, 2);
  covariance[12] = matrix(2, 0);
  covariance[13] = matrix(2, 1);
  covariance[14] = matrix(2, 2);
  return covariance;
}

std::optional<Eigen::Matrix3d> OASIS::AHRS::ParseMatrix3(const std::array<double, 9>& values)
{
  if (!std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); }))
  {
    return std::nullopt;
  }

  Eigen::Matrix3d matrix;
  matrix << values[0], values[1], values[2], values[3], values[4], values[5], values[6], values[7],
      values[8];
  if (matrix(0, 0) < 0.0 || matrix(1, 1) < 0.0 || matrix(2, 2) < 0.0)
    return std::nullopt;

  return matrix;
}

std::optional<Eigen::Matrix3d> OASIS::AHRS::ParseLinearCovariance3(
    const std::array<double, 36>& values)
{
  std::array<double, 9> linear_values{values[0], values[1],  values[2],  values[6], values[7],
                                      values[8], values[12], values[13], values[14]};
  return ParseMatrix3(linear_values);
}

std::optional<AhrsMountingSolution> OASIS::AHRS::SolveMountingFromGravity(
    const Eigen::Vector3d& mean_gravity_unit_imu, int sample_count, double span_sec)
{
  const std::optional<Eigen::Vector3d> normalized =
      NormalizeVector(mean_gravity_unit_imu, kMinGravityNorm);
  if (!normalized.has_value())
    return std::nullopt;

  const double gravity_x = normalized->x();
  const double gravity_y = normalized->y();
  const double gravity_z = normalized->z();

  const double roll_rad = std::atan2(-gravity_y, -gravity_z);
  const double sin_roll = std::sin(roll_rad);
  const double cos_roll = std::cos(roll_rad);

  const double gravity_z_after_roll = sin_roll * gravity_y + cos_roll * gravity_z;
  const double pitch_rad = std::atan2(gravity_x, -gravity_z_after_roll);
  const double yaw_rad = 0.0;

  const std::optional<Eigen::Quaterniond> q_BI =
      NormalizeQuaternion(QuaternionFromRollPitchYaw(roll_rad, pitch_rad, yaw_rad));
  if (!q_BI.has_value())
    return std::nullopt;

  AhrsMountingSolution solution;
  solution.q_BI = *q_BI;
  solution.R_BI = q_BI->toRotationMatrix();
  solution.roll_rad = roll_rad;
  solution.pitch_rad = pitch_rad;
  solution.sample_count = sample_count;
  solution.span_sec = span_sec;
  return solution;
}

std::optional<AhrsGravityResidual> OASIS::AHRS::ComputeGravityResidual(
    const Eigen::Vector3d& measured_gravity_base,
    const std::optional<Eigen::Matrix3d>& measured_gravity_covariance_base,
    const Eigen::Quaterniond& q_WB)
{
  const std::optional<Eigen::Vector3d> measured_direction =
      NormalizeVector(measured_gravity_base, kMinGravityNorm);
  if (!measured_direction.has_value())
    return std::nullopt;

  AhrsGravityResidual residual;
  residual.measured_direction = *measured_direction;
  const Eigen::Quaterniond q_BW = q_WB.conjugate();
  residual.predicted_direction = q_BW.toRotationMatrix() * Eigen::Vector3d(0.0, 0.0, -1.0);
  residual.residual_vector = residual.measured_direction - residual.predicted_direction;
  residual.residual_norm = residual.residual_vector.norm();

  if (measured_gravity_covariance_base.has_value())
  {
    const double measured_norm = measured_gravity_base.norm();
    const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();

    // Units: 1 / (m/s^2)
    // Meaning: Jacobian of u = g / |g|
    // Derivation: du/dg = (I - u u^T) / |g|
    const Eigen::Matrix3d jacobian =
        (identity - residual.measured_direction * residual.measured_direction.transpose()) /
        measured_norm;
    Eigen::Matrix3d normalized_covariance =
        jacobian * (*measured_gravity_covariance_base) * jacobian.transpose();
    normalized_covariance += kNormalizedCovarianceFloor * identity;

    const std::optional<Eigen::Matrix3d> inverse = InvertMatrix3(normalized_covariance);
    if (inverse.has_value())
    {
      const double quadratic =
          residual.residual_vector.transpose() * (*inverse) * residual.residual_vector;
      if (std::isfinite(quadratic) && quadratic >= 0.0)
        residual.mahalanobis_distance = std::sqrt(quadratic);
    }
  }

  return residual;
}

std::optional<Eigen::Matrix3d> OASIS::AHRS::GravityCovarianceToRollPitchCovariance(
    const Eigen::Vector3d& gravity_base,
    const std::optional<Eigen::Matrix3d>& gravity_covariance_base,
    double yaw_variance_rad2)
{
  if (!gravity_covariance_base.has_value())
    return std::nullopt;

  if (!IsFiniteVector3(gravity_base) || !IsFiniteMatrix3(*gravity_covariance_base))
    return std::nullopt;

  const double gravity_x = gravity_base.x();
  const double gravity_y = gravity_base.y();
  const double gravity_z = gravity_base.z();
  const double gravity_norm = gravity_base.norm();
  if (gravity_norm < kMinAttitudeCovarianceGravityNorm)
    return std::nullopt;

  const double lateral_norm = std::hypot(gravity_y, gravity_z);
  if (lateral_norm < kMinRollPitchObservableNorm)
    return std::nullopt;

  Eigen::Matrix3d covariance = *gravity_covariance_base;
  covariance = 0.5 * (covariance + covariance.transpose());
  if (covariance(0, 0) < 0.0 || covariance(1, 1) < 0.0 || covariance(2, 2) < 0.0)
    return std::nullopt;

  const double lateral_norm2 = lateral_norm * lateral_norm;
  const double gravity_norm2 = gravity_norm * gravity_norm;
  const Eigen::RowVector3d roll_jacobian(0.0, gravity_z / lateral_norm2,
                                         -gravity_y / lateral_norm2);
  const Eigen::RowVector3d pitch_jacobian(lateral_norm / gravity_norm2,
                                          -gravity_x * gravity_y / (lateral_norm * gravity_norm2),
                                          -gravity_x * gravity_z / (lateral_norm * gravity_norm2));

  const double roll_variance = roll_jacobian * covariance * roll_jacobian.transpose();
  const double pitch_variance = pitch_jacobian * covariance * pitch_jacobian.transpose();
  const double roll_pitch_covariance = roll_jacobian * covariance * pitch_jacobian.transpose();

  if (!std::isfinite(roll_variance) || !std::isfinite(pitch_variance) ||
      !std::isfinite(roll_pitch_covariance) || roll_variance < 0.0 || pitch_variance < 0.0 ||
      yaw_variance_rad2 < 0.0)
  {
    return std::nullopt;
  }

  Eigen::Matrix3d orientation_covariance = Eigen::Matrix3d::Zero();
  orientation_covariance(0, 0) = roll_variance;
  orientation_covariance(0, 1) = roll_pitch_covariance;
  orientation_covariance(1, 0) = roll_pitch_covariance;
  orientation_covariance(1, 1) = pitch_variance;
  orientation_covariance(2, 2) = yaw_variance_rad2;
  return orientation_covariance;
}
