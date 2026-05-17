/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/core/Bno086DrainHealth.hpp"

#include <algorithm>

namespace OASIS::IMU::BNO086
{
void Bno086DrainHealth::Record(const Bno086DrainCounters& counters,
                               Bno086DrainAction exit_action,
                               std::uint32_t drain_duration_us)
{
  ++m_drains;
  m_physicalPacketsSum += counters.physical_packets_this_drain;
  m_physicalPacketsMax = std::max(m_physicalPacketsMax, counters.physical_packets_this_drain);
  m_sensorEventsSum += counters.sensor_events_this_drain;
  m_sensorEventsMax = std::max(m_sensorEventsMax, counters.sensor_events_this_drain);
  m_drainDurationSumUs += drain_duration_us;
  m_drainDurationMaxUs = std::max(m_drainDurationMaxUs, drain_duration_us);

  if (exit_action == Bno086DrainAction::PhysicalPacketCap)
    ++m_physicalPacketCapHitCount;
  if (exit_action == Bno086DrainAction::PollIterationCap)
    ++m_pollIterationCapHitCount;
  if (exit_action == Bno086DrainAction::AllZeroBudget)
    ++m_allZeroBudgetHitCount;
  if (exit_action == Bno086DrainAction::NoProgressBudget)
    ++m_noProgressDrainCount;
  if (exit_action == Bno086DrainAction::TransportError)
    ++m_transportErrorCount;
}

void Bno086DrainHealth::CountAllZeroBackoff()
{
  ++m_allZeroBackoffCount;
}

bool Bno086DrainHealth::HasSafetyFailure() const
{
  return m_physicalPacketCapHitCount > 0 || m_pollIterationCapHitCount > 0 ||
         m_noProgressDrainCount > 0 || m_transportErrorCount > 0 || m_allZeroBudgetHitCount > 0;
}

std::uint64_t Bno086DrainHealth::Drains() const
{
  return m_drains;
}

std::uint32_t Bno086DrainHealth::PhysicalPacketsMax() const
{
  return m_physicalPacketsMax;
}

std::uint32_t Bno086DrainHealth::SensorEventsMax() const
{
  return m_sensorEventsMax;
}

std::uint64_t Bno086DrainHealth::AllZeroBackoffCount() const
{
  return m_allZeroBackoffCount;
}

std::uint32_t Bno086DrainHealth::DrainDurationMaxUs() const
{
  return m_drainDurationMaxUs;
}

std::uint64_t Bno086DrainHealth::PhysicalPacketCapHitCount() const
{
  return m_physicalPacketCapHitCount;
}

std::uint64_t Bno086DrainHealth::PollIterationCapHitCount() const
{
  return m_pollIterationCapHitCount;
}

std::uint64_t Bno086DrainHealth::NoProgressDrainCount() const
{
  return m_noProgressDrainCount;
}

std::uint64_t Bno086DrainHealth::TransportErrorCount() const
{
  return m_transportErrorCount;
}

std::uint64_t Bno086DrainHealth::AllZeroBudgetHitCount() const
{
  return m_allZeroBudgetHitCount;
}

double Bno086DrainHealth::PhysicalPacketsMean() const
{
  return m_drains > 0 ? static_cast<double>(m_physicalPacketsSum) / static_cast<double>(m_drains)
                      : 0.0;
}

double Bno086DrainHealth::SensorEventsMean() const
{
  return m_drains > 0 ? static_cast<double>(m_sensorEventsSum) / static_cast<double>(m_drains)
                      : 0.0;
}

double Bno086DrainHealth::DrainDurationMeanMs() const
{
  return m_drains > 0
             ? static_cast<double>(m_drainDurationSumUs) / static_cast<double>(m_drains) / 1.0e3
             : 0.0;
}
} // namespace OASIS::IMU::BNO086
