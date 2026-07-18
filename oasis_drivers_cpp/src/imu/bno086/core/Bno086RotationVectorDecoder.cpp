/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/core/Bno086RotationVectorDecoder.hpp"

#include "imu/bno086/utils/Bno086MathUtils.hpp"

namespace OASIS::IMU::BNO086
{
std::optional<std::array<double, 4>> DecodeRotationVectorWorldFromImu(const SensorEvent& event)
{
  if (event.report_id != ReportId::RotationVector)
    return std::nullopt;

  std::array<double, 4> q_WI{
      QToDouble(event.values[static_cast<std::size_t>(RotationVectorValueIndex::QuaternionI)], 14),
      QToDouble(event.values[static_cast<std::size_t>(RotationVectorValueIndex::QuaternionJ)], 14),
      QToDouble(event.values[static_cast<std::size_t>(RotationVectorValueIndex::QuaternionK)], 14),
      QToDouble(event.values[static_cast<std::size_t>(RotationVectorValueIndex::QuaternionReal)],
                14),
  };
  NormalizeQuaternion(q_WI);
  return q_WI;
}
} // namespace OASIS::IMU::BNO086
