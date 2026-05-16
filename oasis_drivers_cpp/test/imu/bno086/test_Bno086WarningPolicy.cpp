/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/Bno086WarningPolicy.hpp"

#include <gtest/gtest.h>

namespace OASIS::IMU::BNO086
{
namespace
{
Bno086HealthSnapshot HealthySnapshot()
{
  Bno086HealthSnapshot health;
  health.has_rate_window = true;
  health.expected_accel_rate_hz = 125.0;
  health.imu_gravity_rate_hz = 124.0;
  health.accel_rate_hz = 125.0;
  health.orientation_age_ms = 6.0;
  health.gyro_age_ms = 6.0;
  health.max_orientation_age_ms = 25.0;
  health.max_gyro_age_ms = 25.0;
  return health;
}
} // namespace

TEST(Bno086WarningPolicy, healthyDurationBudgetAssertedExitDoesNotWarn)
{
  Bno086DrainExitWarningContext context;
  context.health = HealthySnapshot();
  context.hintn_asserted_at_exit = true;
  context.exited_duration_budget = true;
  context.drain_duration_ms = 50.0;
  context.max_drain_duration_ms = 50.0;

  EXPECT_FALSE(ShouldWarnBno086DrainExit(context));
}

TEST(Bno086WarningPolicy, unhealthyDurationBudgetAssertedExitWarns)
{
  Bno086DrainExitWarningContext context;
  context.health = HealthySnapshot();
  context.health.imu_gravity_rate_hz = 50.0;
  context.hintn_asserted_at_exit = true;
  context.exited_duration_budget = true;
  context.drain_duration_ms = 50.0;
  context.max_drain_duration_ms = 50.0;

  EXPECT_TRUE(ShouldWarnBno086DrainExit(context));
}

TEST(Bno086WarningPolicy, maxPacketAssertedExitWarnsEvenIfHealthy)
{
  Bno086DrainExitWarningContext context;
  context.health = HealthySnapshot();
  context.hintn_asserted_at_exit = true;
  context.exited_max_packets = true;

  EXPECT_TRUE(ShouldWarnBno086DrainExit(context));
}

TEST(Bno086WarningPolicy, successfulLongPollWithHealthyRatesDoesNotWarn)
{
  Bno086PollWarningContext context;
  context.health = HealthySnapshot();
  context.outcome = Bno086PollOutcome::SensorEvent;
  context.poll_duration_ms = 18.0;
  context.long_poll_threshold_ms = 10.0;
  context.severe_poll_duration_ms = 50.0;

  EXPECT_FALSE(ShouldWarnBno086LongPoll(context));
}

TEST(Bno086WarningPolicy, longPollTimeoutWarns)
{
  Bno086PollWarningContext context;
  context.health = HealthySnapshot();
  context.outcome = Bno086PollOutcome::Timeout;
  context.poll_duration_ms = 18.0;
  context.long_poll_threshold_ms = 10.0;
  context.severe_poll_duration_ms = 50.0;

  EXPECT_TRUE(ShouldWarnBno086LongPoll(context));
}

TEST(Bno086WarningPolicy, longPollTransportErrorWarns)
{
  Bno086PollWarningContext context;
  context.health = HealthySnapshot();
  context.outcome = Bno086PollOutcome::TransportError;
  context.poll_duration_ms = 18.0;
  context.long_poll_threshold_ms = 10.0;
  context.severe_poll_duration_ms = 50.0;

  EXPECT_TRUE(ShouldWarnBno086LongPoll(context));
}

TEST(Bno086WarningPolicy, calibratedAccelActual125ForRequested100DoesNotWarn)
{
  Bno086FeatureRateWarningContext context;
  context.requested_rate_hz = 100.0;
  context.actual_rate_hz = 125.0;

  EXPECT_FALSE(ShouldWarnBno086FeatureRate(context));
}

TEST(Bno086WarningPolicy, actualRateBelowToleranceWarns)
{
  Bno086FeatureRateWarningContext context;
  context.requested_rate_hz = 100.0;
  context.actual_rate_hz = 75.0;

  EXPECT_TRUE(ShouldWarnBno086FeatureRate(context));
}

TEST(Bno086WarningPolicy, timestampRepairBelowThresholdDoesNotWarn)
{
  Bno086TimestampRepairWarningContext context;
  context.repair_offset_ns = 1;
  context.repair_delta = 1;
  context.warn_offset_ns = 25'000'000;
  context.warn_delta_per_window = 10;

  EXPECT_FALSE(ShouldWarnBno086TimestampRepair(context));
}

TEST(Bno086WarningPolicy, timestampRepairOffsetAboveThresholdWarns)
{
  Bno086TimestampRepairWarningContext context;
  context.repair_offset_ns = 25'000'001;
  context.repair_delta = 1;
  context.warn_offset_ns = 25'000'000;
  context.warn_delta_per_window = 10;

  EXPECT_TRUE(ShouldWarnBno086TimestampRepair(context));
}

TEST(Bno086WarningPolicy, startupChannel0ContinuationResetDoesNotWarn)
{
  Bno086ContinuationResetWarningContext context;
  context.health = HealthySnapshot();
  context.channel = 0;
  context.reset_delta = 1;
  context.communication_established = false;
  context.startup_backlog = true;

  EXPECT_FALSE(ShouldWarnBno086ContinuationReset(context));
}

TEST(Bno086WarningPolicy, repeatedContinuationResetsWarn)
{
  Bno086ContinuationResetWarningContext context;
  context.health = HealthySnapshot();
  context.channel = 3;
  context.reset_delta = 4;
  context.communication_established = true;
  context.warn_delta_per_window = 3;

  EXPECT_TRUE(ShouldWarnBno086ContinuationReset(context));
}
} // namespace OASIS::IMU::BNO086
