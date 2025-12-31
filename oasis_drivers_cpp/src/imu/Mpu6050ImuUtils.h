/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <stdint.h>

namespace OASIS::IMU
{
class Mpu6050ImuUtils
{
public:
  static double AccelScaleFromRange(uint8_t range);
  static double GyroScaleFromRange(uint8_t range);

private:
  double m_accelScale{0.0};
  double m_gyroScale{0.0};
};
} // namespace OASIS::IMU
