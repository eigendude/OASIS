/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/diagnostics/Bno086RateHealth.hpp"

#include <chrono>

#include <gtest/gtest.h>

using namespace OASIS::IMU::BNO086;

TEST(Bno086RateHealth, computesDecodedReportRates)
{
  Bno086RateHealth health;
  const auto start = Bno086RateHealth::Clock::time_point(std::chrono::seconds(1));
  health.BuildSnapshot(start);
  health.MarkSnapshotLogged(start);

  health.CountDecodedReport(ReportId::Accelerometer);
  health.CountDecodedReport(ReportId::Accelerometer);
  health.CountDecodedReport(ReportId::GyroscopeCalibrated);

  const Bno086RateSnapshot snapshot = health.BuildSnapshot(start + std::chrono::seconds(1));

  EXPECT_DOUBLE_EQ(snapshot.decoded_hz[0], 2.0);
  EXPECT_DOUBLE_EQ(snapshot.decoded_hz[1], 1.0);
}

TEST(Bno086RateHealth, computesPublicationRates)
{
  Bno086RateHealth health;
  const auto start = Bno086RateHealth::Clock::time_point(std::chrono::seconds(2));
  health.BuildSnapshot(start);
  health.MarkSnapshotLogged(start);

  health.CountImuPublished();
  health.CountImuPublished();
  health.CountImuGravityPublished();

  const Bno086RateSnapshot snapshot = health.BuildSnapshot(start + std::chrono::milliseconds(500));

  EXPECT_DOUBLE_EQ(snapshot.imu_hz, 4.0);
  EXPECT_DOUBLE_EQ(snapshot.imu_gravity_hz, 2.0);
}

TEST(Bno086RateHealth, detectsLowRatesWithMinimumFraction)
{
  Bno086RateHealth health;
  Bno086RateSnapshot snapshot;
  snapshot.decoded_hz[0] = 20.0;
  snapshot.imu_gravity_hz = 20.0;

  const Bno086ExpectedRates expected{50.0, 50.0, 50.0, 50.0, 25.0};

  EXPECT_TRUE(health.HasRateFailure(snapshot, expected, 0.5));

  snapshot.decoded_hz[0] = 25.0;
  snapshot.imu_gravity_hz = 25.0;
  EXPECT_FALSE(health.HasRateFailure(snapshot, expected, 0.5));
}
