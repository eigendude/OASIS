/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "environment/Bme280Device.h"
#include "environment/Ens160Device.h"
#include "environment/RunningVariance.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include <oasis_msgs/msg/air_quality_index.hpp>
#include <oasis_msgs/msg/gas_concentration.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/relative_humidity.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_msgs/msg/header.hpp>

namespace OASIS::ROS
{
class Ens160Bme280Node : public rclcpp::Node
{
public:
  explicit Ens160Bme280Node(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~Ens160Bme280Node() override = default;

private:
  struct Config
  {
    //! Linux I2C device path
    std::string i2c_device;
    //! ENS160 7-bit I2C address, 0x52 or 0x53
    std::uint8_t ens160_i2c_address{0x53};
    //! BME280 7-bit I2C address, 0x76 or 0x77
    std::uint8_t bme280_i2c_address{0x77};
    //! ROS frame assigned to all output message headers
    std::string frame_id;
    //! Sampling and publication frequency in hertz
    double sample_rate_hz{1.0};
    //! Maximum samples retained by each rolling variance estimator
    std::size_t variance_window_samples{60};
    //! Whether ENS160 warm-up and initial-startup output may be published
    bool publish_during_warmup{true};
  };

  Config ReadConfig();
  bool InitializeEns160();
  bool InitializeBme280();
  void Sample();
  void HandleBme280Failure();
  void HandleEns160Failure();
  void PublishBme280(const OASIS::Environment::Bme280Sample& sample,
                     const std_msgs::msg::Header& header);
  void PublishEns160(const OASIS::Environment::Ens160Sample& sample,
                     const std_msgs::msg::Header& header);
  void ResetBme280Variances();
  void ResetEns160Variances();

  Config m_config;
  OASIS::Environment::Bme280Device m_bme280;
  OASIS::Environment::Ens160Device m_ens160;
  bool m_bme280Initialized{false};
  bool m_ens160Initialized{false};
  bool m_haveCompensation{false};
  unsigned int m_bme280Failures{0};
  unsigned int m_ens160Failures{0};

  OASIS::Environment::RunningVariance m_aqiVariance;
  OASIS::Environment::RunningVariance m_eco2Variance;
  OASIS::Environment::RunningVariance m_tvocVariance;
  OASIS::Environment::RunningVariance m_temperatureVariance;
  OASIS::Environment::RunningVariance m_humidityVariance;
  OASIS::Environment::RunningVariance m_pressureVariance;

  rclcpp::Publisher<oasis_msgs::msg::AirQualityIndex>::SharedPtr m_aqiPublisher;
  rclcpp::Publisher<oasis_msgs::msg::GasConcentration>::SharedPtr m_eco2Publisher;
  rclcpp::Publisher<oasis_msgs::msg::GasConcentration>::SharedPtr m_tvocPublisher;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr m_temperaturePublisher;
  rclcpp::Publisher<sensor_msgs::msg::RelativeHumidity>::SharedPtr m_humidityPublisher;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr m_pressurePublisher;
  rclcpp::TimerBase::SharedPtr m_timer;
};
} // namespace OASIS::ROS
