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
} // namespace OASIS::IMU::BNO086
