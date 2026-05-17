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
#include <optional>

namespace OASIS::IMU::BNO086
{
/*!
 * \brief Gravity payload prepared for the ROS `gravity` topic
 *
 * The linear vector uses the canonical OASIS convention:
 * - expressed in `imu_link`
 * - points in the direction of gravitational acceleration
 * - magnitude is near 9.81 m/s^2 at rest
 *
 * The rotational covariance block is always marked unknown because the driver
 * does not estimate angular acceleration for this topic.
 */
struct PublishedGravityMeasurement
{
  Vec3 gravity_mps2{0.0, 0.0, 0.0};
  std::array<double, 36> covariance{};
};

/*!
 * \brief Convert the raw SH-2 gravity vector into canonical OASIS gravity
 *
 * SH-2 gravity is treated here as the opposite sign of the OASIS convention.
 * OASIS stores and publishes gravity as a physical down vector in `imu_link`.
 */
Vec3 CanonicalizeGravityVector(const Vec3& raw_gravity_mps2);

/*!
 * \brief Build the linear+covariance payload for the ROS `gravity` topic
 */
PublishedGravityMeasurement MakePublishedGravityMeasurement(
    const Vec3& gravity_mps2, const std::optional<Mat3>& gravity_cov_mps2_2);
} // namespace OASIS::IMU::BNO086
