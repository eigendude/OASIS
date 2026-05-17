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
#include <cstdint>

namespace OASIS::IMU::BNO086
{
double QToDouble(std::int16_t value, unsigned q_point);
void NormalizeQuaternion(std::array<double, 4>& q);
std::array<double, 4> MultiplyQuaternion(const std::array<double, 4>& lhs,
                                         const std::array<double, 4>& rhs);
std::array<double, 4> PredictQuaternion(const std::array<double, 4>& orientation_xyzw,
                                        const OASIS::IMU::Vec3& gyro_rads,
                                        double prediction_horizon_sec);
} // namespace OASIS::IMU::BNO086
