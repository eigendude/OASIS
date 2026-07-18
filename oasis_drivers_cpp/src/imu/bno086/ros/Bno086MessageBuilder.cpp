/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/ros/Bno086MessageBuilder.hpp"

#include "imu/bno086/core/Bno086GravityUtils.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>

namespace
{
void SetCovariance(std::array<double, 9>& dst, const OASIS::IMU::Mat3& src)
{
  dst[0] = src[0][0];
  dst[1] = src[0][1];
  dst[2] = src[0][2];

  dst[3] = src[1][0];
  dst[4] = src[1][1];
  dst[5] = src[1][2];

  dst[6] = src[2][0];
  dst[7] = src[2][1];
  dst[8] = src[2][2];
}

void NormalizeQuaternion(std::array<double, 4>& q)
{
  const double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);

  if (norm <= 0.0)
  {
    q = {0.0, 0.0, 0.0, 1.0};
    return;
  }

  q[0] /= norm;
  q[1] /= norm;
  q[2] /= norm;
  q[3] /= norm;
}

std::array<double, 4> MultiplyQuaternion(const std::array<double, 4>& lhs,
                                         const std::array<double, 4>& rhs)
{
  return {
      lhs[3] * rhs[0] + lhs[0] * rhs[3] + lhs[1] * rhs[2] - lhs[2] * rhs[1],
      lhs[3] * rhs[1] - lhs[0] * rhs[2] + lhs[1] * rhs[3] + lhs[2] * rhs[0],
      lhs[3] * rhs[2] + lhs[0] * rhs[1] - lhs[1] * rhs[0] + lhs[2] * rhs[3],
      lhs[3] * rhs[3] - lhs[0] * rhs[0] - lhs[1] * rhs[1] - lhs[2] * rhs[2],
  };
}

std::array<double, 4> PredictQuaternion(
    const std::array<double, 4>& orientation_world_from_imu_xyzw,
    const OASIS::IMU::Vec3& gyro_rads,
    double prediction_horizon_sec)
{
  if (prediction_horizon_sec <= 0.0)
    return orientation_world_from_imu_xyzw;

  const double angularSpeed = std::sqrt(gyro_rads[0] * gyro_rads[0] + gyro_rads[1] * gyro_rads[1] +
                                        gyro_rads[2] * gyro_rads[2]);
  if (angularSpeed <= 1e-9)
    return orientation_world_from_imu_xyzw;

  const double halfAngle = 0.5 * angularSpeed * prediction_horizon_sec;
  const double sinHalfAngle = std::sin(halfAngle);
  const double invAngularSpeed = 1.0 / angularSpeed;
  std::array<double, 4> deltaQuaternion{
      gyro_rads[0] * invAngularSpeed * sinHalfAngle,
      gyro_rads[1] * invAngularSpeed * sinHalfAngle,
      gyro_rads[2] * invAngularSpeed * sinHalfAngle,
      std::cos(halfAngle),
  };

  std::array<double, 4> predictedOrientation =
      MultiplyQuaternion(orientation_world_from_imu_xyzw, deltaQuaternion);
  NormalizeQuaternion(predictedOrientation);
  return predictedOrientation;
}

std::array<double, 9> PredictedCovarianceFromPresent(
    const std::array<double, 9>& present_orientation_covariance,
    double prediction_horizon_sec,
    double& sigma_noise_rad,
    double& sigma_rms_rad,
    double& sigma_bound_rad)
{
  // Present orientation covariance comes from the driver-owned SH-2 policy.
  // Host prediction should never be more confident than that estimate, so
  // start from the present-time diagonal and add only nonnegative growth.
  //
  // The growth term models additional small-angle variance accumulated while
  // integrating gyro forward in time:
  //
  //   sigma_growth = kPredictionSigmaRateRadPerSec * horizon
  //   variance_growth = sigma_growth^2
  //
  // This keeps Sigma_pred(h=0) == Sigma_present and makes the diagonal grow
  // monotonically for h > 0.
  constexpr double kPredictionSigmaRateRadPerSec = 0.05;

  std::array<double, 9> predictedCovariance{};
  predictedCovariance.fill(0.0);

  const double clampedHorizonSec = std::max(prediction_horizon_sec, 0.0);
  sigma_noise_rad = kPredictionSigmaRateRadPerSec * clampedHorizonSec;

  const double growthVarianceRad2 = sigma_noise_rad * sigma_noise_rad;
  double maxPredictedVarianceRad2 = 0.0;

  for (std::size_t axis = 0; axis < 3; ++axis)
  {
    const std::size_t diagonalIndex = (axis * 3) + axis;
    const double presentVarianceRad2 = std::max(present_orientation_covariance[diagonalIndex], 0.0);
    const double predictedVarianceRad2 = presentVarianceRad2 + growthVarianceRad2;
    predictedCovariance[diagonalIndex] = predictedVarianceRad2;
    maxPredictedVarianceRad2 = std::max(maxPredictedVarianceRad2, predictedVarianceRad2);
  }

  sigma_rms_rad = std::sqrt(maxPredictedVarianceRad2);
  sigma_bound_rad = sigma_rms_rad;
  return predictedCovariance;
}
} // namespace

namespace OASIS::ROS
{
sensor_msgs::msg::Imu BuildBno086PresentImuMessage(
    const Bno086RosMessageConfig& config,
    const OASIS::IMU::BNO086::ImuSampleFrame& latest_frame,
    const rclcpp::Time& stamp)
{
  sensor_msgs::msg::Imu imuMsg;
  imuMsg.header.stamp = stamp;
  imuMsg.header.frame_id = config.frame_id;

  imuMsg.orientation.x = latest_frame.orientation_world_from_imu_xyzw[0];
  imuMsg.orientation.y = latest_frame.orientation_world_from_imu_xyzw[1];
  imuMsg.orientation.z = latest_frame.orientation_world_from_imu_xyzw[2];
  imuMsg.orientation.w = latest_frame.orientation_world_from_imu_xyzw[3];

  imuMsg.angular_velocity.x = latest_frame.gyro_rads[0];
  imuMsg.angular_velocity.y = latest_frame.gyro_rads[1];
  imuMsg.angular_velocity.z = latest_frame.gyro_rads[2];

  imuMsg.linear_acceleration.x = latest_frame.linear_accel_mps2[0];
  imuMsg.linear_acceleration.y = latest_frame.linear_accel_mps2[1];
  imuMsg.linear_acceleration.z = latest_frame.linear_accel_mps2[2];

  imuMsg.orientation_covariance.fill(0.0);
  imuMsg.angular_velocity_covariance.fill(0.0);
  imuMsg.linear_acceleration_covariance.fill(0.0);

  if (latest_frame.has_orientation_covariance)
    SetCovariance(imuMsg.orientation_covariance, latest_frame.orientation_cov_rad2);

  if (latest_frame.has_gyro_covariance)
    SetCovariance(imuMsg.angular_velocity_covariance, latest_frame.gyro_cov_rads2_2);

  if (latest_frame.has_linear_accel_covariance)
    SetCovariance(imuMsg.linear_acceleration_covariance, latest_frame.linear_accel_cov_mps2_2);

  return imuMsg;
}

sensor_msgs::msg::Imu BuildBno086ImuGravityMessage(
    const Bno086RosMessageConfig& config,
    const OASIS::IMU::BNO086::ImuSampleFrame& latest_frame,
    const OASIS::IMU::BNO086::Bno086ImuGravityAccelSample& accel_sample,
    const rclcpp::Time& stamp)
{
  sensor_msgs::msg::Imu imuGravityMsg;
  imuGravityMsg.header.stamp = stamp;
  imuGravityMsg.header.frame_id = config.frame_id;

  imuGravityMsg.orientation.x = latest_frame.orientation_world_from_imu_xyzw[0];
  imuGravityMsg.orientation.y = latest_frame.orientation_world_from_imu_xyzw[1];
  imuGravityMsg.orientation.z = latest_frame.orientation_world_from_imu_xyzw[2];
  imuGravityMsg.orientation.w = latest_frame.orientation_world_from_imu_xyzw[3];

  imuGravityMsg.angular_velocity.x = latest_frame.gyro_rads[0];
  imuGravityMsg.angular_velocity.y = latest_frame.gyro_rads[1];
  imuGravityMsg.angular_velocity.z = latest_frame.gyro_rads[2];

  imuGravityMsg.linear_acceleration.x = accel_sample.accel_mps2[0];
  imuGravityMsg.linear_acceleration.y = accel_sample.accel_mps2[1];
  imuGravityMsg.linear_acceleration.z = accel_sample.accel_mps2[2];

  imuGravityMsg.orientation_covariance.fill(0.0);
  imuGravityMsg.angular_velocity_covariance.fill(0.0);
  imuGravityMsg.linear_acceleration_covariance.fill(0.0);

  if (latest_frame.has_orientation_covariance)
    SetCovariance(imuGravityMsg.orientation_covariance, latest_frame.orientation_cov_rad2);

  if (latest_frame.has_gyro_covariance)
    SetCovariance(imuGravityMsg.angular_velocity_covariance, latest_frame.gyro_cov_rads2_2);

  if (accel_sample.has_covariance)
    SetCovariance(imuGravityMsg.linear_acceleration_covariance, accel_sample.covariance_mps2_2);

  return imuGravityMsg;
}

geometry_msgs::msg::AccelWithCovarianceStamped BuildBno086GravityMessage(
    const Bno086RosMessageConfig& config,
    const OASIS::IMU::BNO086::ImuSampleFrame& latest_frame,
    const rclcpp::Time& stamp)
{
  std::optional<OASIS::IMU::Mat3> gravityCovariance;
  if (latest_frame.has_gravity_covariance)
    gravityCovariance = latest_frame.gravity_cov_mps2_2;

  const OASIS::IMU::BNO086::PublishedGravityMeasurement gravityMeasurement =
      OASIS::IMU::BNO086::MakePublishedGravityMeasurement(latest_frame.gravity_mps2,
                                                          gravityCovariance);

  geometry_msgs::msg::AccelWithCovarianceStamped gravityMsg;
  gravityMsg.header.stamp = stamp;
  gravityMsg.header.frame_id = config.frame_id;
  gravityMsg.accel.accel.linear.x = gravityMeasurement.gravity_mps2[0];
  gravityMsg.accel.accel.linear.y = gravityMeasurement.gravity_mps2[1];
  gravityMsg.accel.accel.linear.z = gravityMeasurement.gravity_mps2[2];

  gravityMsg.accel.accel.angular.x = 0.0;
  gravityMsg.accel.accel.angular.y = 0.0;
  gravityMsg.accel.accel.angular.z = 0.0;

  // `gravity` publishes the canonical OASIS gravity vector in
  // `accel.accel.linear`: expressed in `imu_link`, pointing down, and near
  // 9.81 m/s^2 at rest. This is a physical gravity vector, not an "up"
  // vector and not a normalized direction-only unit vector.
  //
  // The rotational covariance block in geometry_msgs/AccelWithCovariance is
  // reserved for angular acceleration. The BNO086 does not estimate angular
  // acceleration on this topic, so covariance[21] remains -1.0 by policy.
  gravityMsg.accel.covariance = gravityMeasurement.covariance;

  return gravityMsg;
}

sensor_msgs::msg::Imu BuildBno086PredictedImuMessage(
    const Bno086RosMessageConfig& config,
    const OASIS::IMU::BNO086::ImuSampleFrame& latest_frame,
    const sensor_msgs::msg::Imu& present_imu)
{
  sensor_msgs::msg::Imu predictedImuMsg = present_imu;
  const std::array<double, 4> predictedOrientation =
      PredictQuaternion(latest_frame.orientation_world_from_imu_xyzw, latest_frame.gyro_rads,
                        config.prediction_horizon_sec);

  predictedImuMsg.orientation.x = predictedOrientation[0];
  predictedImuMsg.orientation.y = predictedOrientation[1];
  predictedImuMsg.orientation.z = predictedOrientation[2];
  predictedImuMsg.orientation.w = predictedOrientation[3];

  double sigmaNoiseRad = 0.0;
  double sigmaRmsRad = 0.0;
  double sigmaBoundRad = 0.0;
  predictedImuMsg.orientation_covariance = PredictedCovarianceFromPresent(
      present_imu.orientation_covariance, config.prediction_horizon_sec, sigmaNoiseRad, sigmaRmsRad,
      sigmaBoundRad);

  return predictedImuMsg;
}

oasis_msgs::msg::ImuVr BuildBno086PredictedVrMessage(
    const Bno086RosMessageConfig& config,
    const Bno086RosPredictionState& prediction_state,
    const sensor_msgs::msg::Imu& present_imu,
    const sensor_msgs::msg::Imu& predicted_imu)
{
  oasis_msgs::msg::ImuVr vrMsg;
  const std::uint8_t predictionAccuracy =
      std::min(prediction_state.orientation_accuracy, prediction_state.gyro_accuracy);
  vrMsg.header = present_imu.header;
  vrMsg.valid = prediction_state.has_orientation && prediction_state.has_gyro;
  vrMsg.source = config.prediction_source;
  vrMsg.prediction_horizon_sec = config.prediction_horizon_sec;
  vrMsg.orientation = predicted_imu.orientation;
  vrMsg.orientation_covariance = predicted_imu.orientation_covariance;
  vrMsg.accuracy_status = predictionAccuracy;
  vrMsg.covariance_is_prediction_model_based = config.prediction_horizon_sec > 0.0;

  double sigmaNoiseRad = 0.0;
  double sigmaRmsRad = 0.0;
  double sigmaBoundRad = 0.0;
  PredictedCovarianceFromPresent(present_imu.orientation_covariance, config.prediction_horizon_sec,
                                 sigmaNoiseRad, sigmaRmsRad, sigmaBoundRad);
  vrMsg.sigma_noise_rad = sigmaNoiseRad;
  vrMsg.sigma_rms_rad = sigmaRmsRad;
  vrMsg.sigma_bound_rad = sigmaBoundRad;

  vrMsg.angular_velocity = present_imu.angular_velocity;
  vrMsg.angular_velocity_covariance = present_imu.angular_velocity_covariance;
  vrMsg.linear_acceleration = present_imu.linear_acceleration;
  vrMsg.linear_acceleration_covariance = present_imu.linear_acceleration_covariance;
  return vrMsg;
}
} // namespace OASIS::ROS
