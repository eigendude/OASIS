/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/Bno086DiagnosticsPolicy.hpp"

namespace OASIS::IMU::BNO086
{
std::optional<Bno086DiagnosticsLogLevel> ParseBno086DiagnosticsLogLevel(std::string_view level)
{
  if (level == "debug")
    return Bno086DiagnosticsLogLevel::Debug;

  if (level == "info")
    return Bno086DiagnosticsLogLevel::Info;

  if (level == "warn")
    return Bno086DiagnosticsLogLevel::Warn;

  if (level == "off")
    return Bno086DiagnosticsLogLevel::Off;

  return std::nullopt;
}

bool ShouldEmitBno086Diagnostics(Bno086DiagnosticsLogLevel level, bool unhealthy)
{
  switch (level)
  {
    case Bno086DiagnosticsLogLevel::Debug:
    case Bno086DiagnosticsLogLevel::Info:
      return true;
    case Bno086DiagnosticsLogLevel::Warn:
      return unhealthy;
    case Bno086DiagnosticsLogLevel::Off:
      return false;
    default:
      break;
  }

  return false;
}
} // namespace OASIS::IMU::BNO086
