/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <optional>
#include <string_view>

namespace OASIS::IMU::BNO086
{
/*!\brief Periodic BNO086 diagnostics logging level */
enum class Bno086DiagnosticsLogLevel
{
  Debug,
  Info,
  Warn,
  Off,
};

std::optional<Bno086DiagnosticsLogLevel> ParseBno086DiagnosticsLogLevel(std::string_view level);

bool ShouldEmitBno086Diagnostics(Bno086DiagnosticsLogLevel level, bool unhealthy);
} // namespace OASIS::IMU::BNO086
