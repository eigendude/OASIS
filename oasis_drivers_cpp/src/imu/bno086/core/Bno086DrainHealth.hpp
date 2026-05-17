/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "imu/bno086/core/Bno086DrainPolicy.hpp"

#include <cstdint>

namespace OASIS::IMU::BNO086
{
/*!
 * \brief Aggregated BNO086 interrupt drain health counters
 */
class Bno086DrainHealth
{
public:
  void Record(const Bno086DrainCounters& counters,
              Bno086DrainAction exit_action,
              std::uint32_t drain_duration_us);

  void CountAllZeroBackoff();

  bool HasSafetyFailure() const;

  std::uint64_t Drains() const;
  std::uint32_t PhysicalPacketsMax() const;
  std::uint32_t SensorEventsMax() const;
  std::uint64_t AllZeroBackoffCount() const;
  std::uint32_t DrainDurationMaxUs() const;
  std::uint64_t PhysicalPacketCapHitCount() const;
  std::uint64_t PollIterationCapHitCount() const;
  std::uint64_t NoProgressDrainCount() const;
  std::uint64_t TransportErrorCount() const;
  std::uint64_t AllZeroBudgetHitCount() const;
  double PhysicalPacketsMean() const;
  double SensorEventsMean() const;
  double DrainDurationMeanMs() const;

private:
  std::uint64_t m_drains{0};
  std::uint64_t m_physicalPacketsSum{0};
  std::uint32_t m_physicalPacketsMax{0};
  std::uint64_t m_sensorEventsSum{0};
  std::uint32_t m_sensorEventsMax{0};
  std::uint64_t m_allZeroBackoffCount{0};
  std::uint64_t m_drainDurationSumUs{0};
  std::uint32_t m_drainDurationMaxUs{0};
  std::uint64_t m_physicalPacketCapHitCount{0};
  std::uint64_t m_pollIterationCapHitCount{0};
  std::uint64_t m_noProgressDrainCount{0};
  std::uint64_t m_transportErrorCount{0};
  std::uint64_t m_allZeroBudgetHitCount{0};
};
} // namespace OASIS::IMU::BNO086
