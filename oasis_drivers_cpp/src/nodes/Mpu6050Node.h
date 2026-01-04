/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "imu/ImuProcessor.h"
#include "imu/ImuTemperature.h"
#include "imu/io/ImuCalibrationFile.h"

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>

#include <MPU6050.h>
#include <oasis_msgs/msg/imu_calibration.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_msgs/msg/header.hpp>

namespace OASIS
{
namespace ROS
{

class Mpu6050Node : public rclcpp::Node
{
public:
  Mpu6050Node();

  // Lifecycle functions
  bool Initialize();
  void Deinitialize();

private:
  // Timer target
  void PublishImu();

  // Helper functions
  oasis_msgs::msg::ImuCalibration ToImuCalibrationMsg(const std_msgs::msg::Header& header,
                                                      const IMU::ImuCalibrationRecord& rec) const;

  // ROS publishers
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_imuPublisher;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_imuRawPublisher;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr m_imuTemperaturePublisher;
  rclcpp::Publisher<oasis_msgs::msg::ImuCalibration>::SharedPtr m_imuCalibrationPublisher;

  // ROS timers
  rclcpp::TimerBase::SharedPtr m_timer;

  // ROS parameters
  std::string m_i2cDevice;
  std::chrono::duration<double> m_publishPeriod;
  double m_gravity;
  std::string m_systemId;
  std::string m_imuCalibrationBase;

  // Calibration
  std::filesystem::path m_calibrationCachePath;
  IMU::ImuCalibrationFile m_calibFile;
  rclcpp::Time m_lastCalibrationSaveTime;
  bool m_hasLastCalibrationSaveTime{false};

  bool m_hasBestCalibration{false};
  double m_bestRmsMps2{0.0};
  std::uint32_t m_bestFitSampleCount{0};

  // IMU parameters
  std::unique_ptr<MPU6050> m_mpu6050;
  IMU::ImuProcessor m_imuProcessor;
  IMU::ImuTemperature m_imuTemperature;
  std::optional<rclcpp::Time> m_lastSampleTime;
};

} // namespace ROS
} // namespace OASIS
