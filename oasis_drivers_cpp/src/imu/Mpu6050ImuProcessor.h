/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

namespace OASIS::IMU
{
class Mpu6050ImuProcessor
{
public:
  Mpu6050ImuProcessor();
  ~Mpu6050ImuProcessor();

  void SetAccelScale(double accelScale);
  void SetGyroScale(double gyroScale);

private:
  double m_accelScale = 0.0;
  double m_gyroScale = 0.0;
};
} // namespace OASIS::IMU
