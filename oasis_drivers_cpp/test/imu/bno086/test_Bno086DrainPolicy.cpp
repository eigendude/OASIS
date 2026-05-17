/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/Bno086DrainPolicy.hpp"

#include <gtest/gtest.h>

using namespace OASIS::IMU::BNO086;

namespace
{
Bno086Shtp::PollResult SensorEventResult(bool physical_packet, bool pending_event)
{
  Bno086Shtp::PollResult result;
  result.status = Bno086Shtp::PollStatus::SensorEvent;
  result.event = SensorEvent{};
  result.read_physical_packet = physical_packet;
  result.dequeued_pending_event = pending_event;
  return result;
}

Bno086Shtp::PollResult TimeoutResult()
{
  Bno086Shtp::PollResult result;
  result.status = Bno086Shtp::PollStatus::Timeout;
  return result;
}
} // namespace

TEST(Bno086DrainPolicy, pendingDecodedEventsDoNotIncrementPhysicalPacketCount)
{
  Bno086DrainLimits limits;
  Bno086DrainCounters counters;

  const Bno086DrainDecision decision =
      Bno086DrainAfterPoll(SensorEventResult(false, true), limits, counters, true);

  EXPECT_EQ(decision.action, Bno086DrainAction::Continue);
  EXPECT_EQ(counters.poll_iterations, 1U);
  EXPECT_EQ(counters.physical_packets_this_drain, 0U);
  EXPECT_EQ(counters.sensor_events_this_drain, 1U);
  EXPECT_EQ(counters.pending_events_this_drain, 1U);
}

TEST(Bno086DrainPolicy, physicalPacketReadsIncrementPhysicalPacketCount)
{
  Bno086DrainLimits limits;
  Bno086DrainCounters counters;

  const Bno086DrainDecision decision =
      Bno086DrainAfterPoll(SensorEventResult(true, false), limits, counters, true);

  EXPECT_EQ(decision.action, Bno086DrainAction::Continue);
  EXPECT_EQ(counters.poll_iterations, 1U);
  EXPECT_EQ(counters.physical_packets_this_drain, 1U);
  EXPECT_EQ(counters.sensor_events_this_drain, 1U);
  EXPECT_EQ(counters.pending_events_this_drain, 0U);
}

TEST(Bno086DrainPolicy, timeoutAfterProgressContinuesWhenHintnAsserted)
{
  Bno086DrainLimits limits;
  Bno086DrainCounters counters;

  ASSERT_EQ(Bno086DrainAfterPoll(SensorEventResult(true, false), limits, counters, true).action,
            Bno086DrainAction::Continue);

  const Bno086DrainDecision decision =
      Bno086DrainAfterPoll(TimeoutResult(), limits, counters, true);

  EXPECT_EQ(decision.action, Bno086DrainAction::Continue);
  EXPECT_TRUE(decision.hintn_asserted);
  EXPECT_EQ(counters.poll_iterations, 2U);
  EXPECT_EQ(counters.consecutive_no_progress_polls, 1U);
}

TEST(Bno086DrainPolicy, timeoutBeforeProgressWithHintnAssertedContinuesWithinBudget)
{
  Bno086DrainLimits limits;
  Bno086DrainCounters counters;

  const Bno086DrainDecision decision =
      Bno086DrainAfterPoll(TimeoutResult(), limits, counters, true);

  EXPECT_EQ(decision.action, Bno086DrainAction::Continue);
  EXPECT_TRUE(decision.hintn_asserted);
  EXPECT_EQ(counters.poll_iterations, 1U);
  EXPECT_EQ(counters.consecutive_no_progress_polls, 1U);
}

TEST(Bno086DrainPolicy, assertedHintnTimeoutExitsWhenNoProgressBudgetIsExhausted)
{
  Bno086DrainLimits limits;
  limits.max_no_progress_polls_per_interrupt = 2;
  Bno086DrainCounters counters;

  ASSERT_EQ(Bno086DrainAfterPoll(TimeoutResult(), limits, counters, true).action,
            Bno086DrainAction::Continue);

  const Bno086DrainDecision decision =
      Bno086DrainAfterPoll(TimeoutResult(), limits, counters, true);

  EXPECT_EQ(decision.action, Bno086DrainAction::NoProgressBudget);
  EXPECT_TRUE(decision.hintn_asserted);
  EXPECT_EQ(counters.poll_iterations, 2U);
  EXPECT_EQ(counters.consecutive_no_progress_polls, 2U);
}

TEST(Bno086DrainPolicy, physicalProgressResetsNoProgressBudget)
{
  Bno086DrainLimits limits;
  limits.max_no_progress_polls_per_interrupt = 2;
  Bno086DrainCounters counters;

  ASSERT_EQ(Bno086DrainAfterPoll(TimeoutResult(), limits, counters, true).action,
            Bno086DrainAction::Continue);
  ASSERT_EQ(Bno086DrainAfterPoll(SensorEventResult(true, false), limits, counters, true).action,
            Bno086DrainAction::Continue);

  const Bno086DrainDecision decision =
      Bno086DrainAfterPoll(TimeoutResult(), limits, counters, true);

  EXPECT_EQ(decision.action, Bno086DrainAction::Continue);
  EXPECT_EQ(counters.consecutive_no_progress_polls, 1U);
}

TEST(Bno086DrainPolicy, timeoutBeforeProgressWithHintnDeassertedCompletes)
{
  Bno086DrainLimits limits;
  Bno086DrainCounters counters;

  const Bno086DrainDecision decision =
      Bno086DrainAfterPoll(TimeoutResult(), limits, counters, false);

  EXPECT_EQ(decision.action, Bno086DrainAction::Complete);
  EXPECT_FALSE(decision.hintn_asserted);
  EXPECT_EQ(counters.poll_iterations, 1U);
}

TEST(Bno086DrainPolicy, physicalPacketCapUsesPhysicalPacketCountNotEventCount)
{
  Bno086DrainLimits limits;
  limits.max_physical_packets_per_interrupt = 2;
  Bno086DrainCounters counters;

  ASSERT_EQ(Bno086DrainAfterPoll(SensorEventResult(false, true), limits, counters, true).action,
            Bno086DrainAction::Continue);
  ASSERT_EQ(Bno086DrainAfterPoll(SensorEventResult(false, true), limits, counters, true).action,
            Bno086DrainAction::Continue);

  const Bno086DrainDecision decision =
      Bno086DrainAfterPoll(SensorEventResult(true, false), limits, counters, true);

  EXPECT_EQ(decision.action, Bno086DrainAction::Continue);
  EXPECT_EQ(counters.sensor_events_this_drain, 3U);
  EXPECT_EQ(counters.pending_events_this_drain, 2U);
  EXPECT_EQ(counters.physical_packets_this_drain, 1U);

  const Bno086DrainDecision cappedDecision =
      Bno086DrainAfterPoll(SensorEventResult(true, false), limits, counters, true);

  EXPECT_EQ(cappedDecision.action, Bno086DrainAction::PhysicalPacketCap);
  EXPECT_TRUE(cappedDecision.hintn_asserted);
  EXPECT_EQ(counters.sensor_events_this_drain, 4U);
  EXPECT_EQ(counters.physical_packets_this_drain, 2U);
}

TEST(Bno086DrainPolicy, pollIterationCapPreventsInfiniteLoops)
{
  Bno086DrainLimits limits;
  limits.max_poll_iterations_per_interrupt = 2;
  Bno086DrainCounters counters;

  ASSERT_EQ(Bno086DrainAfterPoll(SensorEventResult(false, true), limits, counters, true).action,
            Bno086DrainAction::Continue);
  ASSERT_EQ(Bno086DrainAfterPoll(SensorEventResult(false, true), limits, counters, true).action,
            Bno086DrainAction::Continue);

  const Bno086DrainDecision decision = Bno086DrainBeforePoll(limits, counters, true);

  EXPECT_EQ(decision.action, Bno086DrainAction::PollIterationCap);
  EXPECT_TRUE(decision.hintn_asserted);
}

TEST(Bno086DrainPolicy, sensorEventBudgetBoundsOneDrain)
{
  Bno086DrainLimits limits;
  limits.max_sensor_events_per_drain = 2;
  Bno086DrainCounters counters;

  ASSERT_EQ(Bno086DrainAfterPoll(SensorEventResult(true, false), limits, counters, true).action,
            Bno086DrainAction::Continue);

  const Bno086DrainDecision decision =
      Bno086DrainAfterPoll(SensorEventResult(true, false), limits, counters, true);

  EXPECT_EQ(decision.action, Bno086DrainAction::SensorEventBudget);
  EXPECT_TRUE(decision.hintn_asserted);
  EXPECT_EQ(counters.sensor_events_this_drain, 2U);
  EXPECT_EQ(counters.pending_events_this_drain, 0U);
}

TEST(Bno086DrainPolicy, sensorEventBudgetDoesNotInterruptPendingFlush)
{
  Bno086DrainLimits limits;
  limits.max_sensor_events_per_drain = 1;
  limits.max_pending_events_flush_per_drain = 3;
  Bno086DrainCounters counters;

  const Bno086DrainDecision decision =
      Bno086DrainAfterPoll(SensorEventResult(false, true), limits, counters, true);

  EXPECT_EQ(decision.action, Bno086DrainAction::Continue);
  EXPECT_EQ(counters.sensor_events_this_drain, 1U);
  EXPECT_EQ(counters.pending_events_this_drain, 1U);
}

TEST(Bno086DrainPolicy, pendingEventFlushBudgetBoundsOneDrain)
{
  Bno086DrainLimits limits;
  limits.max_pending_events_flush_per_drain = 2;
  Bno086DrainCounters counters;

  ASSERT_EQ(Bno086DrainAfterPoll(SensorEventResult(false, true), limits, counters, true).action,
            Bno086DrainAction::Continue);

  const Bno086DrainDecision decision =
      Bno086DrainAfterPoll(SensorEventResult(false, true), limits, counters, true);

  EXPECT_EQ(decision.action, Bno086DrainAction::PendingEventFlushBudget);
  EXPECT_TRUE(decision.hintn_asserted);
  EXPECT_EQ(counters.sensor_events_this_drain, 2U);
  EXPECT_EQ(counters.pending_events_this_drain, 2U);
}
