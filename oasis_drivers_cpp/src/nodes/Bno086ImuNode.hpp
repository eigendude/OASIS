/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "imu/bno086/Bno086Gpio.hpp"
#include "imu/bno086/Bno086Reports.hpp"
#include "imu/bno086/Bno086Shtp.hpp"
#include "imu/bno086/Bno086Transport.hpp"

#include <atomic>
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace OASIS::ROS
{
/*!
 * \brief BNO086 IMU driver node
 *
 * Publishes:
 * - imu: fused orientation + calibrated gyro + gravity-removed linear accel
 * - gravity (optional): gravity vector from BNO086 fusion output
 * - accel (optional): gravity-included calibrated acceleration
 *
 * Publication is interrupt-driven from active-low H_INTN packet drains
 * report_rate_hz configures BNO086 internal report timing only
 *
 * Orientation uses the BNO086 Rotation Vector output, which is
 * magnetometer-backed internally for heading. Magnetometer is not published.
 */
class Bno086ImuNode : public rclcpp::Node
{
public:
  Bno086ImuNode();

  bool Initialize();
  void Deinitialize();

private:
  struct SampleState
  {
    bool has_sample{false};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    std::uint8_t sequence{0};
  };

  struct CoreFrameSignature
  {
    std::uint8_t orientation_sequence{0};
    std::uint8_t gyro_sequence{0};
    std::uint8_t linear_accel_sequence{0};
    int64_t orientation_stamp_ns{0};
    int64_t gyro_stamp_ns{0};
    int64_t linear_accel_stamp_ns{0};
  };

  void InterruptLoop();
  void DrainPacketsForInterrupt(const std::chrono::steady_clock::time_point& interrupt_steady_at,
                                const rclcpp::Time& interrupt_ros_at);
  void MaybePublishOnLinearAcceleration(const OASIS::IMU::BNO086::SensorEvent& event);
  void PublishLatestFrame(const rclcpp::Time& stamp);
  std::uint32_t CoreCoherenceToleranceUs() const;
  bool HasPublishableCoreFrame() const;
  CoreFrameSignature LatestCoreSignature() const;
  rclcpp::Time LatestCoreStamp() const;

  void ApplyEvent(const OASIS::IMU::BNO086::SensorEvent& event, const rclcpp::Time& sample_stamp);
  rclcpp::Time EstimateEventStamp(const OASIS::IMU::BNO086::SensorEvent& event,
                                  const rclcpp::Time& interrupt_ros_at);

  static OASIS::IMU::Mat3 CovarianceFromAccuracy(std::uint8_t accuracy,
                                                 double sigma_unreliable,
                                                 double sigma_low,
                                                 double sigma_medium,
                                                 double sigma_high);

  static double QToDouble(std::int16_t value, unsigned q_point);
  static void NormalizeQuaternion(std::array<double, 4>& q);
  static void SetCovariance(std::array<double, 9>& dst, const OASIS::IMU::Mat3& src);

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_imuPublisher;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr m_gravityPublisher;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr m_accelPublisher;

  OASIS::IMU::BNO086::Bno086Transport m_transport;
  OASIS::IMU::BNO086::Bno086Gpio m_interruptGpio;
  std::unique_ptr<OASIS::IMU::BNO086::Bno086Shtp> m_shtp;
  OASIS::IMU::BNO086::ImuSampleFrame m_latestFrame;
  SampleState m_orientationState{};
  SampleState m_gyroState{};
  SampleState m_linearAccelState{};

  std::string m_frameId;

  std::atomic<bool> m_running{false};
  std::thread m_interruptThread;

  std::optional<std::uint32_t> m_lastBaseTimestampUs;
  std::optional<rclcpp::Time> m_lastBaseRosStamp;
  std::optional<CoreFrameSignature> m_lastPublishedCoreSignature;
  std::uint32_t m_reportIntervalUs{10'000};

  bool m_loggedCommEstablished{false};
  bool m_loggedSetFeature{false};
  bool m_warnedMissingImuFields{false};
};
} // namespace OASIS::ROS
