/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/Bno086DrainDiagnostics.hpp"

#include <gtest/gtest.h>

namespace OASIS::IMU::BNO086
{
TEST(Bno086DrainDiagnostics, timeoutWithHintnDeassertedExitsCleanly)
{
  InterruptDrainDiagnostics diagnostics;

  const TimeoutRetryDecision decision = HandleTimeoutWhileDraining(diagnostics, false, 0, 3);

  EXPECT_FALSE(decision.retry);
  EXPECT_TRUE(decision.exit_timeout);
  EXPECT_FALSE(diagnostics.latest_timeout_hintn_asserted);
  EXPECT_EQ(diagnostics.timeout_retries_while_hintn_asserted, 0U);
  EXPECT_EQ(diagnostics.drain_exited_timeout, 1U);
  EXPECT_EQ(diagnostics.drain_exited_timeout_after_progress, 0U);
  EXPECT_EQ(diagnostics.drain_exited_timeout_no_progress, 1U);
  EXPECT_EQ(diagnostics.drain_exited_timeout_hintn_deasserted, 1U);
  EXPECT_EQ(diagnostics.drain_exited_timeout_hintn_asserted, 0U);
}

TEST(Bno086DrainDiagnostics, timeoutWithHintnAssertedRetriesWithinBudget)
{
  InterruptDrainDiagnostics diagnostics;

  const TimeoutRetryDecision decision = HandleTimeoutWhileDraining(diagnostics, true, 0, 3);

  EXPECT_TRUE(decision.retry);
  EXPECT_FALSE(decision.exit_timeout);
  EXPECT_TRUE(diagnostics.latest_timeout_hintn_asserted);
  EXPECT_EQ(diagnostics.timeout_retries_while_hintn_asserted, 1U);
  EXPECT_EQ(diagnostics.drain_exited_timeout, 0U);
  EXPECT_EQ(diagnostics.drain_exited_timeout_after_progress, 0U);
  EXPECT_EQ(diagnostics.drain_exited_timeout_no_progress, 0U);
  EXPECT_EQ(diagnostics.drain_exited_timeout_hintn_asserted, 0U);
}

TEST(Bno086DrainDiagnostics, assertedTimeoutAfterRetryBudgetCountsAssertedExit)
{
  InterruptDrainDiagnostics diagnostics;

  const TimeoutRetryDecision decision = HandleTimeoutWhileDraining(diagnostics, true, 3, 3);

  EXPECT_FALSE(decision.retry);
  EXPECT_TRUE(decision.exit_timeout);
  EXPECT_TRUE(diagnostics.latest_timeout_hintn_asserted);
  EXPECT_EQ(diagnostics.timeout_retries_while_hintn_asserted, 0U);
  EXPECT_EQ(diagnostics.drain_exited_timeout, 1U);
  EXPECT_EQ(diagnostics.drain_exited_timeout_after_progress, 0U);
  EXPECT_EQ(diagnostics.drain_exited_timeout_no_progress, 1U);
  EXPECT_EQ(diagnostics.drain_exited_timeout_hintn_deasserted, 0U);
  EXPECT_EQ(diagnostics.drain_exited_timeout_hintn_asserted, 1U);
}

TEST(Bno086DrainDiagnostics, timeoutAfterProgressExitsWithoutRetry)
{
  InterruptDrainDiagnostics diagnostics;

  const TimeoutRetryDecision decision = HandleDrainTimeout(diagnostics, true, true, 0, 3);

  EXPECT_FALSE(decision.retry);
  EXPECT_TRUE(decision.exit_timeout);
  EXPECT_TRUE(diagnostics.latest_timeout_hintn_asserted);
  EXPECT_EQ(diagnostics.timeout_retries_while_hintn_asserted, 0U);
  EXPECT_EQ(diagnostics.drain_exited_timeout, 1U);
  EXPECT_EQ(diagnostics.drain_exited_timeout_after_progress, 1U);
  EXPECT_EQ(diagnostics.drain_exited_timeout_no_progress, 0U);
  EXPECT_EQ(diagnostics.drain_exited_timeout_hintn_deasserted, 0U);
  EXPECT_EQ(diagnostics.drain_exited_timeout_hintn_asserted, 1U);
}

TEST(Bno086DrainDiagnostics, consecutiveAssertedDrainExitsAreTracked)
{
  InterruptDrainDiagnostics diagnostics;

  RecordDrainExitHintnState(diagnostics, true, 5);
  RecordDrainExitHintnState(diagnostics, true, 5);

  EXPECT_TRUE(diagnostics.latest_hintn_asserted_at_exit);
  EXPECT_EQ(diagnostics.count_hintn_still_asserted_at_exit, 2U);
  EXPECT_EQ(diagnostics.consecutive_hintn_asserted_drain_exits, 2U);
  EXPECT_EQ(diagnostics.max_consecutive_hintn_asserted_drain_exits, 2U);
  EXPECT_EQ(diagnostics.stuck_interrupt_recovery_candidate_count, 0U);
}

TEST(Bno086DrainDiagnostics, consecutiveAssertedDrainExitsResetAfterCleanExit)
{
  InterruptDrainDiagnostics diagnostics;

  RecordDrainExitHintnState(diagnostics, true, 5);
  RecordDrainExitHintnState(diagnostics, true, 5);
  RecordDrainExitHintnState(diagnostics, false, 5);

  EXPECT_FALSE(diagnostics.latest_hintn_asserted_at_exit);
  EXPECT_EQ(diagnostics.count_hintn_still_asserted_at_exit, 2U);
  EXPECT_EQ(diagnostics.consecutive_hintn_asserted_drain_exits, 0U);
  EXPECT_EQ(diagnostics.max_consecutive_hintn_asserted_drain_exits, 2U);
}

TEST(Bno086DrainDiagnostics, recoveryCandidateCountsAfterThreshold)
{
  InterruptDrainDiagnostics diagnostics;

  for (int i = 0; i < 6; ++i)
    RecordDrainExitHintnState(diagnostics, true, 5);

  EXPECT_EQ(diagnostics.consecutive_hintn_asserted_drain_exits, 6U);
  EXPECT_EQ(diagnostics.max_consecutive_hintn_asserted_drain_exits, 6U);
  EXPECT_EQ(diagnostics.stuck_interrupt_recovery_candidate_count, 1U);
}

TEST(Bno086DrainDiagnostics, drainDurationUpdatesThroughput)
{
  InterruptDrainDiagnostics diagnostics;

  RecordDrainDuration(diagnostics, 10.0, 25, 100, 50.0);

  EXPECT_DOUBLE_EQ(diagnostics.max_drain_duration_ms_config, 50.0);
  EXPECT_DOUBLE_EQ(diagnostics.latest_drain_duration_ms, 10.0);
  EXPECT_DOUBLE_EQ(diagnostics.max_drain_duration_ms_observed, 10.0);
  EXPECT_DOUBLE_EQ(diagnostics.packets_per_ms_latest, 2.5);
  EXPECT_DOUBLE_EQ(diagnostics.sensor_events_per_ms_latest, 10.0);
}

TEST(Bno086DrainDiagnostics, drainDurationHandlesZeroDuration)
{
  InterruptDrainDiagnostics diagnostics;

  RecordDrainDuration(diagnostics, 0.0, 25, 100, 50.0);

  EXPECT_DOUBLE_EQ(diagnostics.max_drain_duration_ms_config, 50.0);
  EXPECT_DOUBLE_EQ(diagnostics.latest_drain_duration_ms, 0.0);
  EXPECT_DOUBLE_EQ(diagnostics.packets_per_ms_latest, 0.0);
  EXPECT_DOUBLE_EQ(diagnostics.sensor_events_per_ms_latest, 0.0);
}

TEST(Bno086DrainDiagnostics, pollDurationUpdatesThresholdCounters)
{
  InterruptDrainDiagnostics diagnostics;

  RecordPollDuration(diagnostics, 12.0, 5, 1.0);
  RecordPollDuration(diagnostics, 51.0, 5, 1.0);

  EXPECT_EQ(diagnostics.poll_calls, 2U);
  EXPECT_DOUBLE_EQ(diagnostics.latest_poll_duration_ms, 51.0);
  EXPECT_DOUBLE_EQ(diagnostics.max_poll_duration_ms, 51.0);
  EXPECT_EQ(diagnostics.poll_duration_over_timeout_count, 2U);
  EXPECT_EQ(diagnostics.poll_duration_over_10ms_count, 2U);
  EXPECT_EQ(diagnostics.poll_duration_over_50ms_count, 1U);
}

TEST(Bno086DrainDiagnostics, pollDurationWithinSlopDoesNotCountOverTimeout)
{
  InterruptDrainDiagnostics diagnostics;

  RecordPollDuration(diagnostics, 5.5, 5, 1.0);

  EXPECT_EQ(diagnostics.poll_calls, 1U);
  EXPECT_DOUBLE_EQ(diagnostics.latest_poll_duration_ms, 5.5);
  EXPECT_EQ(diagnostics.poll_duration_over_timeout_count, 0U);
  EXPECT_EQ(diagnostics.poll_duration_over_10ms_count, 0U);
  EXPECT_EQ(diagnostics.poll_duration_over_50ms_count, 0U);
}
} // namespace OASIS::IMU::BNO086
