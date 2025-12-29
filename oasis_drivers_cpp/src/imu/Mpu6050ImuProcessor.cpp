/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/Mpu6050ImuProcessor.h"

using namespace OASIS::IMU;

Mpu6050ImuProcessor::Mpu6050ImuProcessor() = default;

void Mpu6050ImuProcessor::SetAccelScale(double accelScale)
{
  m_accelScale = accelScale;
}

void Mpu6050ImuProcessor::SetGyroScale(double gyroScale)
{
  m_gyroScale = gyroScale;
}
