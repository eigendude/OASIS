/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "imu/ImuMath.h"

#include <array>

namespace OASIS::IMU
{
/**
 * @brief Helpers for building IMU orientation quaternions without yaw.
 */
namespace ImuOrientationUtils
{
/**
 * @brief Build a tilt-only quaternion aligning up to world +Z with yaw=0.
 *
 * @param u_hat Unit up vector in sensor frame.
 * @param u_hat_valid True if u_hat is valid.
 * @return Quaternion mapping sensor frame to world frame.
 */
Math::Quaternion TiltQuaternionFromUp(const std::array<double, 3>& u_hat, bool u_hat_valid);
} // namespace ImuOrientationUtils
} // namespace OASIS::IMU
