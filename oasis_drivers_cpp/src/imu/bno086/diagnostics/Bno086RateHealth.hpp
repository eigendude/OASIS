/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "imu/bno086/sh2/Bno086Reports.hpp"

#include <array>
#include <chrono>
#include <cstdint>

namespace OASIS::IMU::BNO086
{
/*!
 * \brief Publication and decoded-report rates over one diagnostics window
 */
struct Bno086RateSnapshot
{
  /*!
   * \brief Whether rates were computed from a positive elapsed time window
   *
   * True after a previous diagnostics snapshot was logged and the current
   * timestamp is later than that snapshot timestamp.
   */
  bool has_elapsed_window{false};

  /*!
   * \brief Elapsed diagnostics window duration
   *
   * Units: ms. Zero means no rate window was available for this snapshot.
   */
  double elapsed_ms{0.0};

  /*!
   * \brief Decoded report rates in diagnostic report order
   *
   * Units: Hz. Order is accel, gyro, rotation, linear acceleration, gravity.
   */
  std::array<double, 5> decoded_hz{};

  /*!
   * \brief Published imu_gravity topic rate
   *
   * Units: Hz
   */
  double imu_gravity_hz{0.0};

  /*!
   * \brief Published imu topic rate
   *
   * Units: Hz
   */
  double imu_hz{0.0};
};

/*!
 * \brief Expected BNO086 report and publication rates for health checks
 */
struct Bno086ExpectedRates
{
  /*!
   * \brief Expected accelerometer report rate
   *
   * Units: Hz
   */
  double accelerometer_hz{0.0};

  /*!
   * \brief Expected calibrated gyro report rate
   *
   * Units: Hz
   */
  double gyro_hz{0.0};

  /*!
   * \brief Expected rotation vector report rate
   *
   * Units: Hz
   */
  double rotation_vector_hz{0.0};

  /*!
   * \brief Expected linear acceleration report rate
   *
   * Units: Hz
   */
  double linear_acceleration_hz{0.0};

  /*!
   * \brief Expected gravity report rate
   *
   * Units: Hz
   */
  double gravity_hz{0.0};
};

/*!
 * \brief Counts BNO086 decoded reports and publication rates
 */
class Bno086RateHealth
{
public:
  using Clock = std::chrono::steady_clock;

  void CountDecodedReport(ReportId report_id);
  void CountImuGravityPublished();
  void CountImuPublished();

  bool ShouldLog(Clock::time_point now, int diagnostics_log_period_ms) const;
  Bno086RateSnapshot BuildSnapshot(Clock::time_point now);
  void MarkSnapshotLogged(Clock::time_point now);

  bool HasRateFailure(const Bno086RateSnapshot& snapshot,
                      const Bno086ExpectedRates& expected_rates,
                      double min_healthy_rate_fraction) const;
  bool HasHealthyImuGravityRate(const Bno086RateSnapshot& snapshot,
                                const Bno086ExpectedRates& expected_rates,
                                double min_healthy_rate_fraction) const;

private:
  std::array<std::uint64_t, 5> m_decodedReportsReceived{};
  std::array<std::uint64_t, 5> m_lastRateDecodedReports{};
  std::uint64_t m_imuGravityPublished{0};
  std::uint64_t m_imuPublished{0};
  std::uint64_t m_lastRateImuGravityPublished{0};
  std::uint64_t m_lastRateImuPublished{0};
  Clock::time_point m_lastLogAt{};
};
} // namespace OASIS::IMU::BNO086
