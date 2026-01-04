/*
 *  Copyright (C) 2025-2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/Mpu6050ImuUtils.h"

#include <numbers>

#include <MPU6050.h>

namespace
{
// Default to +/-250°/s. Converts deg/s to rad/s and divides by 131 LSB/(deg/s).
constexpr double GYRO_SCALE = (std::numbers::pi_v<double> / 180.0) / 131.0;
} // namespace

namespace OASIS::IMU
{
double Mpu6050ImuUtils::AccelScaleFromRange(uint8_t range, double gravity)
{
  switch (range)
  {
    case MPU6050_ACCEL_FS_2:
      return gravity / 16384.0;
    case MPU6050_ACCEL_FS_4:
      return gravity / 8192.0;
    case MPU6050_ACCEL_FS_8:
      return gravity / 4096.0;
    case MPU6050_ACCEL_FS_16:
      return gravity / 2048.0;
    default:
      // Default to +/-2g: 16384 counts per g.
      return gravity / 16384.0;
  }
}

double Mpu6050ImuUtils::GyroScaleFromRange(uint8_t range)
{
  switch (range)
  {
    case MPU6050_GYRO_FS_250:
      return (std::numbers::pi_v<double> / 180.0) / 131.0;
    case MPU6050_GYRO_FS_500:
      return (std::numbers::pi_v<double> / 180.0) / 65.5;
    case MPU6050_GYRO_FS_1000:
      return (std::numbers::pi_v<double> / 180.0) / 32.8;
    case MPU6050_GYRO_FS_2000:
      return (std::numbers::pi_v<double> / 180.0) / 16.4;
    default:
      return GYRO_SCALE;
  }
}
} // namespace OASIS::IMU
