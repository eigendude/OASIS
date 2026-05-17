/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/core/Bno086DrainHealth.hpp"

#include <gtest/gtest.h>

using namespace OASIS::IMU::BNO086;

namespace
{
Bno086DrainCounters Counters(std::uint32_t packets, std::uint32_t events)
{
  Bno086DrainCounters counters;
  counters.physical_packets_this_drain = packets;
  counters.sensor_events_this_drain = events;
  return counters;
}
} // namespace

TEST(Bno086DrainHealth, recordsMeansAndMaxValues)
{
  Bno086DrainHealth health;
  health.Record(Counters(2, 4), Bno086DrainAction::Complete, 1000);
  health.Record(Counters(6, 8), Bno086DrainAction::Complete, 3000);
  health.CountAllZeroBackoff();

  EXPECT_EQ(health.Drains(), 2U);
  EXPECT_DOUBLE_EQ(health.PhysicalPacketsMean(), 4.0);
  EXPECT_EQ(health.PhysicalPacketsMax(), 6U);
  EXPECT_DOUBLE_EQ(health.SensorEventsMean(), 6.0);
  EXPECT_EQ(health.SensorEventsMax(), 8U);
  EXPECT_EQ(health.AllZeroBackoffCount(), 1U);
  EXPECT_DOUBLE_EQ(health.DrainDurationMeanMs(), 2.0);
  EXPECT_EQ(health.DrainDurationMaxUs(), 3000U);
}

TEST(Bno086DrainHealth, durationBudgetIsNotSafetyFailure)
{
  Bno086DrainHealth health;
  health.Record(Counters(1, 1), Bno086DrainAction::DrainDurationBudget, 100'000);

  EXPECT_FALSE(health.HasSafetyFailure());
}

TEST(Bno086DrainHealth, safetyExitsAreSafetyFailures)
{
  const Bno086DrainAction actions[] = {
      Bno086DrainAction::PhysicalPacketCap, Bno086DrainAction::PollIterationCap,
      Bno086DrainAction::AllZeroBudget,     Bno086DrainAction::NoProgressBudget,
      Bno086DrainAction::TransportError,
  };

  for (const Bno086DrainAction action : actions)
  {
    Bno086DrainHealth health;
    health.Record(Counters(1, 1), action, 1000);
    EXPECT_TRUE(health.HasSafetyFailure());
  }
}
