/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "imu/ImuTypes.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace OASIS::IMU::BNO086
{
/*!\brief Sensor report identifiers from SH-2 sensor hub protocol */
enum class ReportId : std::uint8_t
{
  Accelerometer = 0x01,
  GyroscopeCalibrated = 0x02,
  LinearAcceleration = 0x04,
  RotationVector = 0x05,
  Gravity = 0x06,
};

/*!\brief Sensor event decoded from one BNO086 report payload */
struct SensorEvent
{
  /*!
   * \brief Report identifier from the sensor-hub stream
   *
   * Units: enum code
   */
  ReportId report_id{ReportId::Accelerometer};

  /*!
   * \brief Status field from the report header
   *
   * Units: bit-field
   */
  std::uint8_t status{0};

  /*!
   * \brief Sequence number from the sensor report payload
   *
   * Units: counter ticks
   */
  std::uint8_t sequence{0};

  /*!
   * \brief Parsed SH-2 accuracy bits from \ref status
   *
   * Units: enum code in range [0, 3]
   */
  std::uint8_t accuracy{0};

  /*!
   * \brief Delay from event sample time to base timestamp
   *
   * Units: microseconds
   */
  std::uint16_t delay_us{0};

  /*!
   * \brief True when \ref delay_us was present in the payload
   */
  bool has_delay{false};

  /*!
   * \brief Base timestamp associated with this event
   *
   * Units: microseconds on SH-2 timestamp domain
   */
  std::uint32_t base_timestamp_us{0};

  /*!
   * \brief True when \ref base_timestamp_us was present in the payload
   */
  bool has_base_timestamp{false};

  /*!
   * \brief Raw Q-format values carried by this report
   *
   * Units: report specific fixed-point values
   */
  std::array<std::int16_t, 5> values{};
};

/*!
 * \brief Rotation Vector payload slots after the 4-byte SH-2 sensor header
 *
 * The BNO08X datasheet's Rotation Vector metadata declares quaternion samples
 * at Q14 and the additional accuracy field at Q12. The SH-2 Rotation Vector
 * input report then maps those five signed 16-bit values to payload bytes
 * 4-13 as quaternion i/j/k/real followed by the accuracy estimate.
 */
enum class RotationVectorValueIndex : std::size_t
{
  QuaternionI = 0,
  QuaternionJ = 1,
  QuaternionK = 2,
  QuaternionReal = 3,
  AccuracyEstimate = 4,
};

/*!\brief Incremental fused sample cache built from BNO086 events */
struct ImuSampleFrame
{
  /*!
   * \brief True when orientation quaternion is updated
   */
  bool has_orientation{false};

  /*!
   * \brief True when calibrated gyroscope sample is updated
   */
  bool has_gyro{false};

  /*!
   * \brief True when gravity-removed linear acceleration is updated
   */
  bool has_linear_accel{false};

  /*!
   * \brief True when gravity-included acceleration is updated
   */
  bool has_accel{false};

  /*!
   * \brief True when gravity vector is updated
   */
  bool has_gravity{false};

  /*!
   * \brief Orientation quaternion in IMU frame
   *
   * Order: x, y, z, w
   * Units: unitless
   */
  std::array<double, 4> orientation_xyzw{0.0, 0.0, 0.0, 1.0};

  /*!
   * \brief Calibrated angular velocity from BNO086 gyroscope
   *
   * Units: rad/s
   */
  Vec3 gyro_rads{0.0, 0.0, 0.0};

  /*!
   * \brief Gravity-removed acceleration from BNO086 linear accel report
   *
   * Units: m/s^2
   */
  Vec3 linear_accel_mps2{0.0, 0.0, 0.0};

  /*!
   * \brief Gravity-included acceleration from calibrated accel report
   *
   * Units: m/s^2
   */
  Vec3 accel_mps2{0.0, 0.0, 0.0};

  /*!
   * \brief Gravity vector estimated by BNO086 fusion stack
   *
   * Units: m/s^2
   */
  Vec3 gravity_mps2{0.0, 0.0, 0.0};

  /*!
   * \brief Orientation covariance in row-major 3x3 layout
   *
   * Units: rad^2
   */
  Mat3 orientation_cov_rad2{};

  /*!
   * \brief Angular velocity covariance in row-major 3x3 layout
   *
   * Units: (rad/s)^2
   */
  Mat3 gyro_cov_rads2_2{};

  /*!
   * \brief Linear acceleration covariance in row-major 3x3 layout
   *
   * Units: (m/s^2)^2
   */
  Mat3 linear_accel_cov_mps2_2{};

  /*!
   * \brief Gravity covariance in row-major 3x3 layout
   *
   * Units: (m/s^2)^2
   */
  Mat3 gravity_cov_mps2_2{};

  /*!
   * \brief True when orientation covariance is populated
   */
  bool has_orientation_covariance{false};

  /*!
   * \brief True when gyro covariance is populated
   */
  bool has_gyro_covariance{false};

  /*!
   * \brief True when linear acceleration covariance is populated
   */
  bool has_linear_accel_covariance{false};

  /*!
   * \brief True when gravity covariance is populated
   */
  bool has_gravity_covariance{false};
};

constexpr std::size_t kShtpHeaderBytes = 4;
constexpr std::uint8_t kShtpReportBaseTimestamp = 0xFB;
constexpr std::uint8_t kShtpReportTimestampRebase = 0xFA;
constexpr std::uint8_t kShtpSetFeatureCommand = 0xFD;
} // namespace OASIS::IMU::BNO086
