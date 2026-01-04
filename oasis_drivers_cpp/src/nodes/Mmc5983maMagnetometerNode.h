/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "magnetometer/MagnetometerPairProcessor.h"
#include "magnetometer/Mmc5983maDevice.h"
#include "magnetometer/Mmc5983maPairSampler.h"

#include <atomic>
#include <chrono>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

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
  struct SampleSnapshot
  {
    rclcpp::Time stamp{};
    Magnetometer::MagnetometerTickOutput output{};
    bool has_sample{false};
    bool gate_ready{false};
  };

  void SamplerLoop();
  void PublishLatest();
  void PublishSample(const rclcpp::Time& stamp, const Magnetometer::MagnetometerTickOutput& output);

  bool ConfigureDevice();
  std::uint8_t EncodeRawRate(std::uint16_t raw_rate_hz) const;
  double ResolveDatasheetNoiseMg(std::uint8_t bandwidth_mode) const;

  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr m_publisher;
  rclcpp::TimerBase::SharedPtr m_timer;

  Magnetometer::Mmc5983maDevice m_device;
  std::unique_ptr<Magnetometer::Mmc5983maPairSampler> m_sampler;
  std::unique_ptr<Magnetometer::MagnetometerPairProcessor> m_processor;

  std::thread m_samplerThread;
  std::atomic<bool> m_running{false};
  std::mutex m_sampleMutex;
  SampleSnapshot m_latestSample;
  bool m_gateReady{false};

  std::string m_i2cDevice;
  std::string m_frameId;
  std::string m_systemId;
  std::string m_magCalibrationBase;
  std::chrono::duration<double> m_publishPeriod;
  std::uint8_t m_i2cAddress{0};
  std::uint8_t m_bandwidthMode{0};
  std::uint16_t m_rawRateHz{0};
  int m_measurementTimeoutMs{0};
  bool m_calibrationMode{false};
  std::filesystem::path m_calibrationCachePath;
};

} // namespace ROS
} // namespace OASIS
