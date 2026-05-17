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
OASIS::IMU::Mat3 CovarianceFromAccuracyBucket(std::uint8_t accuracy,
                                              double sigma_unreliable,
                                              double sigma_low,
                                              double sigma_medium,
                                              double sigma_high);
} // namespace OASIS::IMU::BNO086
