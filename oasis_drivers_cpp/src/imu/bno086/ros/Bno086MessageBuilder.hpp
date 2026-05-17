/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "imu/bno086/core/Bno086ImuGravityAccelHistory.hpp"
#include "imu/bno086/sh2/Bno086Reports.hpp"

#include <cstdint>
#include <string>

#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <oasis_msgs/msg/imu_vr.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace OASIS::ROS
{
/*!
 * \brief ROS message configuration for BNO086 publishers
 */
struct Bno086RosMessageConfig
{
  /*!
   * \brief ROS frame assigned to message headers
   *
   * Units: frame id string
   */
  std::string frame_id;

  /*!
   * \brief Host prediction horizon applied to predicted orientation
   *
   * Units: seconds, expected range [0, +inf)
   */
  double prediction_horizon_sec{0.0};

  /*!
   * \brief Metadata string describing the prediction source
   *
   * Units: source identifier string
   */
  std::string prediction_source{"rotation_vector_plus_host_prediction"};
};

/*!
 * \brief Minimal sample state needed for predicted VR metadata
 */
struct Bno086RosPredictionState
{
  /*!
   * \brief True when the latest frame contains fused orientation
   */
  bool has_orientation{false};

  /*!
   * \brief True when the latest frame contains calibrated gyro
   */
  bool has_gyro{false};

  /*!
   * \brief SH-2 orientation accuracy status
   *
   * Units: enum code in range [0, 3]
   */
  std::uint8_t orientation_accuracy{0};

  /*!
   * \brief SH-2 gyro accuracy status
   *
   * Units: enum code in range [0, 3]
   */
  std::uint8_t gyro_accuracy{0};
};

sensor_msgs::msg::Imu BuildBno086PresentImuMessage(
    const Bno086RosMessageConfig& config,
    const OASIS::IMU::BNO086::ImuSampleFrame& latest_frame,
    const rclcpp::Time& stamp);

sensor_msgs::msg::Imu BuildBno086ImuGravityMessage(
    const Bno086RosMessageConfig& config,
    const OASIS::IMU::BNO086::ImuSampleFrame& latest_frame,
    const OASIS::IMU::BNO086::Bno086ImuGravityAccelSample& accel_sample,
    const rclcpp::Time& stamp);

geometry_msgs::msg::AccelWithCovarianceStamped BuildBno086GravityMessage(
    const Bno086RosMessageConfig& config,
    const OASIS::IMU::BNO086::ImuSampleFrame& latest_frame,
    const rclcpp::Time& stamp);

sensor_msgs::msg::Imu BuildBno086PredictedImuMessage(
    const Bno086RosMessageConfig& config,
    const OASIS::IMU::BNO086::ImuSampleFrame& latest_frame,
    const sensor_msgs::msg::Imu& present_imu);

oasis_msgs::msg::ImuVr BuildBno086PredictedVrMessage(
    const Bno086RosMessageConfig& config,
    const Bno086RosPredictionState& prediction_state,
    const sensor_msgs::msg::Imu& present_imu,
    const sensor_msgs::msg::Imu& predicted_imu);
} // namespace OASIS::ROS
