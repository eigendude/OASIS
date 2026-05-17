/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/diagnostics/Bno086RateHealth.hpp"

#include "imu/bno086/utils/Bno086ReportUtils.hpp"

#include <chrono>
#include <cstddef>
#include <optional>

namespace OASIS::IMU::BNO086
{
void Bno086RateHealth::CountDecodedReport(ReportId report_id)
{
  const std::optional<std::size_t> reportIndex = DiagnosticReportIndex(report_id);
  if (reportIndex.has_value())
    ++m_decodedReportsReceived[*reportIndex];
}

void Bno086RateHealth::CountImuGravityPublished()
{
  ++m_imuGravityPublished;
}

void Bno086RateHealth::CountImuPublished()
{
  ++m_imuPublished;
}

bool Bno086RateHealth::ShouldLog(Clock::time_point now, int diagnostics_log_period_ms) const
{
  if (m_lastLogAt.time_since_epoch().count() == 0)
    return true;

  const auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_lastLogAt);
  return elapsedMs.count() >= diagnostics_log_period_ms;
}

Bno086RateSnapshot Bno086RateHealth::BuildSnapshot(Clock::time_point now)
{
  Bno086RateSnapshot snapshot;
  if (m_lastLogAt.time_since_epoch().count() != 0)
  {
    const auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_lastLogAt);
    if (elapsedMs.count() > 0)
    {
      const std::uint64_t imuGravityPublishedDelta =
          m_imuGravityPublished - m_lastRateImuGravityPublished;
      const std::uint64_t imuPublishedDelta = m_imuPublished - m_lastRateImuPublished;
      snapshot.imu_gravity_hz = static_cast<double>(imuGravityPublishedDelta) * 1000.0 /
                                static_cast<double>(elapsedMs.count());
      snapshot.imu_hz =
          static_cast<double>(imuPublishedDelta) * 1000.0 / static_cast<double>(elapsedMs.count());

      for (std::size_t i = 0; i < snapshot.decoded_hz.size(); ++i)
      {
        const std::uint64_t decodedDelta =
            m_decodedReportsReceived[i] - m_lastRateDecodedReports[i];
        snapshot.decoded_hz[i] =
            static_cast<double>(decodedDelta) * 1000.0 / static_cast<double>(elapsedMs.count());
      }
    }
  }

  m_lastRateImuGravityPublished = m_imuGravityPublished;
  m_lastRateImuPublished = m_imuPublished;
  m_lastRateDecodedReports = m_decodedReportsReceived;
  return snapshot;
}

void Bno086RateHealth::MarkSnapshotLogged(Clock::time_point now)
{
  m_lastLogAt = now;
}

bool Bno086RateHealth::HasRateFailure(const Bno086RateSnapshot& snapshot,
                                      const Bno086ExpectedRates& expected_rates,
                                      double min_healthy_rate_fraction) const
{
  if (snapshot.decoded_hz[0] > 0.0 &&
      snapshot.decoded_hz[0] < expected_rates.accelerometer_hz * min_healthy_rate_fraction)
    return true;

  if (snapshot.decoded_hz[1] > 0.0 &&
      snapshot.decoded_hz[1] < expected_rates.gyro_hz * min_healthy_rate_fraction)
    return true;

  if (snapshot.decoded_hz[2] > 0.0 &&
      snapshot.decoded_hz[2] < expected_rates.rotation_vector_hz * min_healthy_rate_fraction)
    return true;

  if (snapshot.decoded_hz[3] > 0.0 &&
      snapshot.decoded_hz[3] < expected_rates.linear_acceleration_hz * min_healthy_rate_fraction)
    return true;

  if (snapshot.decoded_hz[4] > 0.0 &&
      snapshot.decoded_hz[4] < expected_rates.gravity_hz * min_healthy_rate_fraction)
    return true;

  return snapshot.imu_gravity_hz > 0.0 &&
         snapshot.imu_gravity_hz < expected_rates.rotation_vector_hz * min_healthy_rate_fraction;
}
} // namespace OASIS::IMU::BNO086
