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
std::array<double, 9> PredictedCovarianceFromPresent(
    const std::array<double, 9>& present_orientation_covariance,
    double prediction_horizon_sec,
    double& sigma_noise_rad,
    double& sigma_rms_rad,
    double& sigma_bound_rad);
OASIS::IMU::Mat3 CovarianceFromAccuracyBucket(std::uint8_t accuracy,
                                              double sigma_unreliable,
                                              double sigma_low,
                                              double sigma_medium,
                                              double sigma_high);
void SetCovariance(std::array<double, 9>& dst, const OASIS::IMU::Mat3& src);
} // namespace OASIS::IMU::BNO086
