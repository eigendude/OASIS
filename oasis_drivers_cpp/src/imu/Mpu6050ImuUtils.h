/*
 *  Copyright (C) 2025-2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <stdint.h>

namespace OASIS::IMU
{
/*!
 * \brief Utility helpers for converting MPU6050 full-scale ranges to physical units.
 *
 * Provides scale factors to convert raw accelerometer counts into m/s^2 and raw gyroscope
 * counts into rad/s based on the configured full-scale range.
 */
class Mpu6050ImuUtils
{
public:
  // Returns the accelerometer scale in meters per second squared per count.
  // Gravity is the local gravitational acceleration (m/s^2) to interpret ±1 g.
  static double AccelScaleFromRange(uint8_t range, double gravity);

  // Returns the gyroscope scale in radians per second per count.
  static double GyroScaleFromRange(uint8_t range);
};
} // namespace OASIS::IMU
