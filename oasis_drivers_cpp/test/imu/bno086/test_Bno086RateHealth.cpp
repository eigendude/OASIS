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

  EXPECT_TRUE(snapshot.has_elapsed_window);
  EXPECT_DOUBLE_EQ(snapshot.elapsed_ms, 1000.0);
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

  EXPECT_TRUE(snapshot.has_elapsed_window);
  EXPECT_DOUBLE_EQ(snapshot.elapsed_ms, 500.0);
  EXPECT_DOUBLE_EQ(snapshot.imu_hz, 4.0);
  EXPECT_DOUBLE_EQ(snapshot.imu_gravity_hz, 2.0);
}

TEST(Bno086RateHealth, detectsLowRatesWithMinimumFraction)
{
  Bno086RateHealth health;
  Bno086RateSnapshot snapshot;
  snapshot.has_elapsed_window = true;
  snapshot.decoded_hz[0] = 20.0;
  snapshot.decoded_hz[2] = 40.0;
  snapshot.imu_gravity_hz = 20.0;

  const Bno086ExpectedRates expected{50.0, 0.0, 40.0, 0.0, 0.0};

  EXPECT_TRUE(health.HasRateFailure(snapshot, expected, 0.5));

  snapshot.decoded_hz[0] = 25.0;
  snapshot.imu_gravity_hz = 25.0;
  EXPECT_FALSE(health.HasRateFailure(snapshot, expected, 0.5));
}

TEST(Bno086RateHealth, treatsZeroImuGravityRateAfterElapsedWindowAsUnhealthy)
{
  Bno086RateHealth health;
  Bno086RateSnapshot snapshot;
  snapshot.has_elapsed_window = true;
  snapshot.elapsed_ms = 1000.0;
  snapshot.decoded_hz[2] = 100.0;
  snapshot.imu_gravity_hz = 0.0;

  const Bno086ExpectedRates expected{0.0, 0.0, 100.0, 0.0, 0.0};

  EXPECT_TRUE(health.HasRateFailure(snapshot, expected, 0.5));
  EXPECT_FALSE(health.HasHealthyImuGravityRate(snapshot, expected, 0.5));
}

TEST(Bno086RateHealth, treatsLowPositiveImuGravityRateAsUnhealthy)
{
  Bno086RateHealth health;
  Bno086RateSnapshot snapshot;
  snapshot.has_elapsed_window = true;
  snapshot.elapsed_ms = 1000.0;
  snapshot.decoded_hz[2] = 100.0;
  snapshot.imu_gravity_hz = 49.0;

  const Bno086ExpectedRates expected{0.0, 0.0, 100.0, 0.0, 0.0};

  EXPECT_TRUE(health.HasRateFailure(snapshot, expected, 0.5));
  EXPECT_FALSE(health.HasHealthyImuGravityRate(snapshot, expected, 0.5));
}

TEST(Bno086RateHealth, treatsHealthyImuGravityRateAsHealthy)
{
  Bno086RateHealth health;
  Bno086RateSnapshot snapshot;
  snapshot.has_elapsed_window = true;
  snapshot.elapsed_ms = 1000.0;
  snapshot.decoded_hz[2] = 100.0;
  snapshot.imu_gravity_hz = 50.0;

  const Bno086ExpectedRates expected{0.0, 0.0, 100.0, 0.0, 0.0};

  EXPECT_FALSE(health.HasRateFailure(snapshot, expected, 0.5));
  EXPECT_TRUE(health.HasHealthyImuGravityRate(snapshot, expected, 0.5));
}

TEST(Bno086RateHealth, ignoresFirstSnapshotWithoutElapsedWindow)
{
  Bno086RateHealth health;
  const auto start = Bno086RateHealth::Clock::time_point(std::chrono::seconds(3));
  const Bno086RateSnapshot snapshot = health.BuildSnapshot(start);

  const Bno086ExpectedRates expected{50.0, 50.0, 100.0, 50.0, 50.0};

  EXPECT_FALSE(snapshot.has_elapsed_window);
  EXPECT_DOUBLE_EQ(snapshot.elapsed_ms, 0.0);
  EXPECT_FALSE(health.HasRateFailure(snapshot, expected, 0.5));
  EXPECT_FALSE(health.HasHealthyImuGravityRate(snapshot, expected, 0.5));
}

TEST(Bno086RateHealth, doesNotTreatZeroImuGravityRateAsRecovered)
{
  Bno086RateHealth health;
  Bno086RateSnapshot snapshot;
  snapshot.has_elapsed_window = true;
  snapshot.elapsed_ms = 1000.0;
  snapshot.decoded_hz[2] = 100.0;
  snapshot.imu_gravity_hz = 0.0;

  const Bno086ExpectedRates expected{0.0, 0.0, 100.0, 0.0, 0.0};

  EXPECT_FALSE(health.HasHealthyImuGravityRate(snapshot, expected, 0.5));
}
