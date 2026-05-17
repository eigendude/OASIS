/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/utils/Bno086ReportUtils.hpp"

namespace OASIS::IMU::BNO086
{
const char* ReportName(ReportId report_id)
{
  switch (report_id)
  {
    case ReportId::Accelerometer:
      return "accelerometer";
    case ReportId::GyroscopeCalibrated:
      return "gyro";
    case ReportId::LinearAcceleration:
      return "linear_acceleration";
    case ReportId::RotationVector:
      return "rotation_vector";
    case ReportId::Gravity:
      return "gravity";
    default:
      break;
  }

  return "unknown";
}

std::optional<std::size_t> DiagnosticReportIndex(ReportId report_id)
{
  switch (report_id)
  {
    case ReportId::Accelerometer:
      return 0;
    case ReportId::GyroscopeCalibrated:
      return 1;
    case ReportId::RotationVector:
      return 2;
    case ReportId::LinearAcceleration:
      return 3;
    case ReportId::Gravity:
      return 4;
    default:
      break;
  }

  return std::nullopt;
}
} // namespace OASIS::IMU::BNO086
