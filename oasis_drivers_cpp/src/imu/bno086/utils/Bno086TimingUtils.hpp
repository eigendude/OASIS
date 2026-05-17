/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <cstdint>
#include <optional>

namespace OASIS::IMU::BNO086
{
std::uint32_t MillisecondsToMicroseconds(double milliseconds);
std::optional<std::uint32_t> RateHzToIntervalUs(double rate_hz);
int64_t DurationNsFromUs(std::uint32_t microseconds);
} // namespace OASIS::IMU::BNO086
