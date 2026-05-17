/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "imu/bno086/sh2/Bno086Reports.hpp"

#include <cstddef> // NOLINT(build/include_order)
#include <optional> // NOLINT(build/include_order)

namespace OASIS::IMU::BNO086
{
const char* ReportName(ReportId report_id);
std::optional<std::size_t> DiagnosticReportIndex(ReportId report_id);
} // namespace OASIS::IMU::BNO086
