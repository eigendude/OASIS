/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "magnetometer/CovarianceEstimator3D.h"
#include "magnetometer/Mmc5983maDevice.h"
#include "magnetometer/io/MagnetometerCalibrationFile.h"

#include <chrono>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>

#include <Eigen/Core>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

namespace OASIS
{
namespace ROS
{

class Mmc5983maMagnetometerNode : public rclcpp::Node
{
public:
  Mmc5983maMagnetometerNode();

  bool Initialize();
  void Deinitialize();

private:
  void PollSensor();
  void PublishSample(const Eigen::Vector3d& field_t, const Eigen::Vector3d& offset_t);

  bool ConfigureDevice();
  std::uint8_t EncodeRawRate(std::uint16_t raw_rate_hz) const;
  double ResolveDatasheetNoiseMg(std::uint8_t bandwidth_mode) const;
  bool MaybeWriteCalibration(const Eigen::Vector3d& offset_t);

  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr m_publisher;
  rclcpp::TimerBase::SharedPtr m_timer;

  Magnetometer::Mmc5983maDevice m_device;
  Magnetometer::CovarianceEstimator3D m_covarianceEstimator;
  Magnetometer::MagnetometerCalibrationFile m_calibrationFile;

  Eigen::Vector3d m_lastSetSample = Eigen::Vector3d::Zero();
  bool m_hasSetSample{false};
  bool m_nextIsReset{false};

  rclcpp::Time m_lastSampleTime;
  bool m_hasLastSampleTime{false};
  rclcpp::Time m_lastCalibrationWriteTime;
  bool m_hasLastCalibrationWrite{false};

  std::filesystem::path m_calibrationPath;
  Eigen::Vector3d m_lastOffset = Eigen::Vector3d::Zero();

  std::string m_i2cDevice;
  std::string m_frameId;
  std::chrono::duration<double> m_publishPeriod;
  std::chrono::duration<double> m_rawPeriod;
  std::uint8_t m_i2cAddress{0};
  std::uint8_t m_bandwidthMode{0};
  std::uint16_t m_rawRateHz{0};
  bool m_calibrationMode{false};
  bool m_continuousCalibration{false};
  std::chrono::duration<double> m_calibrationCooldown;
  double m_datasheetNoiseMg{0.0};
  double m_setResetRepeatabilityMg{0.0};
  double m_stationaryAxisStdThresholdT{0.0};
  double m_stationaryMagnitudeStdThresholdT{0.0};
  double m_priorStrengthSamples{0.0};
  double m_ewmaTauSec{0.0};
};

} // namespace ROS
} // namespace OASIS
