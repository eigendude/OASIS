/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "imu/bno086/sh2/Bno086Reports.hpp"

#include <array>
#include <optional>

namespace OASIS::IMU::BNO086
{
/*!
 * \brief Decode an SH-2 Rotation Vector as q_WI in ROS x/y/z/w order
 *
 * \return Normalized rotation mapping IMU vectors into world coordinates, or
 * nullopt when the event is not a Rotation Vector report
 */
std::optional<std::array<double, 4>> DecodeRotationVectorWorldFromImu(const SensorEvent& event);
} // namespace OASIS::IMU::BNO086
