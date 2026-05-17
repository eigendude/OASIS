/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <cstdint>
#include <string>

#include <rclcpp/node.hpp>

namespace OASIS::ROS
{
constexpr int kBno086DefaultIntGpio = 23;

/*!
 * \brief BNO086 transport configuration loaded from ROS parameters
 */
struct Bno086TransportParams
{
  /*!
   * \brief Linux I2C device path used for BNO086 communication
   *
   * Units: filesystem path string
   */
  std::string i2c_device;

  /*!
   * \brief BNO086 I2C address
   *
   * Units: 7-bit I2C address
   */
  std::uint8_t i2c_address{0x4B};

  /*!
   * \brief GPIO line offset connected to active-low H_INTN
   *
   * Units: GPIO line number, expected range [0, +inf)
   */
  int int_gpio{kBno086DefaultIntGpio};
};

/*!
 * \brief BNO086 driver integration parameters
 */
struct Bno086DriverConfig
{
  /*!
   * \brief ROS frame assigned to BNO086 messages
   *
   * Units: frame id string
   */
  std::string frame_id;

  /*!
   * \brief Host prediction horizon for predicted orientation outputs
   *
   * Units: seconds, expected range [0, +inf)
   */
  double prediction_horizon_sec{0.0};

  /*!
   * \brief Metadata source string used by the VR prediction message
   *
   * Units: source identifier string
   */
  std::string prediction_source{"rotation_vector_plus_host_prediction"};

  /*!
   * \brief Per-poll packet read timeout
   *
   * Units: milliseconds
   */
  int packet_read_timeout_ms{5};

  /*!
   * \brief Startup window for draining feature responses
   *
   * Units: milliseconds
   */
  int feature_response_startup_drain_ms{250};

  /*!
   * \brief Maximum startup packets drained for feature responses
   *
   * Units: packet count
   */
  std::uint32_t feature_response_startup_max_packets{128};

  /*!
   * \brief Timeout before logging the feature summary with missing responses
   *
   * Units: milliseconds
   */
  int feature_summary_timeout_ms{5000};

  /*!
   * \brief Minimum period between BNO086 diagnostic log snapshots
   *
   * Units: milliseconds
   */
  int diagnostics_log_period_ms{5000};

  /*!
   * \brief Backoff after all-zero SHTP headers while H_INTN remains asserted
   *
   * Units: microseconds
   */
  int all_zero_backoff_us{500};

  /*!
   * \brief Maximum interrupt drain duration
   *
   * Units: milliseconds
   */
  int max_drain_duration_ms{100};

  /*!
   * \brief Maximum accel sample age accepted for imu_gravity composition
   *
   * Units: milliseconds
   */
  double imu_gravity_max_orientation_age_ms{80.0};

  /*!
   * \brief Maximum gyro sample age accepted for imu_gravity composition
   *
   * Units: milliseconds
   */
  double imu_gravity_max_gyro_age_ms{80.0};
};

/*!
 * \brief BNO086 SH-2 report configuration loaded from ROS parameters
 */
struct Bno086ReportConfig
{
  /*!
   * \brief Requested rotation vector report rate
   *
   * Units: hertz, clamped to at least 1
   */
  double rotation_vector_rate_hz{100.0};

  /*!
   * \brief Requested calibrated gyro report rate
   *
   * Units: hertz, clamped to at least 1
   */
  double gyro_rate_hz{100.0};

  /*!
   * \brief Requested calibrated accelerometer report rate
   *
   * Units: hertz, clamped to at least 1
   */
  double accelerometer_rate_hz{100.0};

  /*!
   * \brief Requested linear acceleration report rate
   *
   * Units: hertz, clamped to at least 1
   */
  double linear_acceleration_rate_hz{50.0};

  /*!
   * \brief Requested gravity report rate
   *
   * Units: hertz, clamped to at least 1
   */
  double gravity_rate_hz{25.0};

  /*!
   * \brief Requested rotation vector maximum batch interval
   *
   * Units: microseconds
   */
  std::uint32_t rotation_vector_batch_interval_us{0};

  /*!
   * \brief Requested gyro maximum batch interval
   *
   * Units: microseconds
   */
  std::uint32_t gyro_batch_interval_us{0};

  /*!
   * \brief Requested accelerometer maximum batch interval
   *
   * Units: microseconds
   */
  std::uint32_t accelerometer_batch_interval_us{0};

  /*!
   * \brief Requested linear acceleration maximum batch interval
   *
   * Units: microseconds
   */
  std::uint32_t linear_acceleration_batch_interval_us{0};

  /*!
   * \brief Requested gravity maximum batch interval
   *
   * Units: microseconds
   */
  std::uint32_t gravity_batch_interval_us{0};

  /*!
   * \brief True when the linear acceleration report should be enabled
   */
  bool enable_linear_acceleration_report{true};

  /*!
   * \brief True when the gravity report should be enabled
   */
  bool enable_gravity_report{true};
};

/*!
 * \brief Interrupt drain safety limits loaded from ROS parameters
 */
struct Bno086DrainConfig
{
  /*!
   * \brief Maximum physical SHTP packets read during one interrupt drain
   *
   * Units: packet count
   */
  std::uint32_t max_packets_per_interrupt{1024};

  /*!
   * \brief Maximum poll loop iterations during one interrupt drain
   *
   * Units: iteration count
   */
  std::uint32_t max_poll_iterations_per_interrupt{4096};

  /*!
   * \brief Maximum no-progress polls before ending one interrupt drain
   *
   * Units: poll count
   */
  std::uint32_t max_no_progress_polls_per_interrupt{64};

  /*!
   * \brief Maximum all-zero header polls before ending one interrupt drain
   *
   * Units: poll count
   */
  std::uint32_t max_all_zero_polls_per_interrupt{64};

  /*!
   * \brief Maximum decoded sensor events handled during one interrupt drain
   *
   * Units: event count
   */
  std::uint32_t max_sensor_events_per_drain{4096};

  /*!
   * \brief Maximum pending events flushed during one interrupt drain
   *
   * Units: event count
   */
  std::uint32_t max_pending_events_flush_per_drain{1024};
};

/*!
 * \brief Complete BNO086 parameter snapshot loaded by the ROS node
 */
struct Bno086NodeParams
{
  /*!
   * \brief Transport and interrupt-line parameters
   */
  Bno086TransportParams transport;

  /*!
   * \brief Driver integration parameters
   */
  Bno086DriverConfig driver;

  /*!
   * \brief SH-2 report parameters
   */
  Bno086ReportConfig reports;

  /*!
   * \brief Interrupt drain safety parameters
   */
  Bno086DrainConfig drain;
};

void DeclareBno086NodeParameters(rclcpp::Node& node);
Bno086NodeParams LoadBno086NodeParameters(const rclcpp::Node& node);
} // namespace OASIS::ROS
