/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/utils/Bno086TimingUtils.hpp"

#include <algorithm>
#include <cmath>

namespace OASIS::IMU::BNO086
{
std::uint32_t MillisecondsToMicroseconds(double milliseconds)
{
  const double clampedMs = std::max(milliseconds, 0.0);
  return static_cast<std::uint32_t>(std::round(clampedMs * 1000.0));
}

std::optional<std::uint32_t> RateHzToIntervalUs(double rate_hz)
{
  if (!(rate_hz > 0.0))
    return std::nullopt;

  return static_cast<std::uint32_t>(std::max(1.0, std::round(1'000'000.0 / rate_hz)));
}

int64_t DurationNsFromUs(std::uint32_t microseconds)
{
  return static_cast<int64_t>(microseconds) * 1'000;
}
} // namespace OASIS::IMU::BNO086
