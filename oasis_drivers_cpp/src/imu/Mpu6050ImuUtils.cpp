/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/Mpu6050ImuUtils.h"

#include <MPU6050.h>

namespace
{
constexpr double PI = 3.14159265358979323846;
constexpr double GRAVITY = 9.80665; // m/s^2
constexpr double ACCEL_SCALE = GRAVITY / 16384.0; // Default to +/-2g
constexpr double GYRO_SCALE = (PI / 180.0) / 131.0; // Default to +/-250°/s
} // namespace

namespace OASIS::IMU
{
double Mpu6050ImuUtils::AccelScaleFromRange(uint8_t range)
{
  switch (range)
  {
    case MPU6050_ACCEL_FS_2:
      return GRAVITY / 16384.0;
    case MPU6050_ACCEL_FS_4:
      return GRAVITY / 8192.0;
    case MPU6050_ACCEL_FS_8:
      return GRAVITY / 4096.0;
    case MPU6050_ACCEL_FS_16:
      return GRAVITY / 2048.0;
    default:
      return ACCEL_SCALE;
  }
}

double Mpu6050ImuUtils::GyroScaleFromRange(uint8_t range)
{
  switch (range)
  {
    case MPU6050_GYRO_FS_250:
      return (PI / 180.0) / 131.0;
    case MPU6050_GYRO_FS_500:
      return (PI / 180.0) / 65.5;
    case MPU6050_GYRO_FS_1000:
      return (PI / 180.0) / 32.8;
    case MPU6050_GYRO_FS_2000:
      return (PI / 180.0) / 16.4;
    default:
      return GYRO_SCALE;
  }
}
} // namespace OASIS::IMU
