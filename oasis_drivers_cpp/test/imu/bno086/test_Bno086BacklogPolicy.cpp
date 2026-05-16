/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/Bno086BacklogPolicy.hpp"

#include <gtest/gtest.h>

namespace OASIS::IMU::BNO086
{
namespace
{
Bno086BacklogSample HealthySample()
{
  Bno086BacklogSample sample;
  sample.events_per_packet_mean = 2.0;
  sample.full_packet_read_bytes_mean = 31.0;
  sample.imu_gravity_rate_hz = 125.0;
  sample.accel_decoded_rate_hz = 125.0;
  sample.latest_drain_duration_ms = 10.0;
  sample.max_drain_duration_ms = 50.0;
  return sample;
}

Bno086BacklogSample BacklogSample()
{
  Bno086BacklogSample sample;
  sample.events_per_packet_mean = 7.3;
  sample.full_packet_read_bytes_mean = 90.0;
  sample.imu_gravity_rate_hz = 20.0;
  sample.accel_decoded_rate_hz = 125.0;
  sample.accel_sequence_gap_delta = 47;
  sample.stale_orientation_skip_delta = 5;
  sample.latest_drain_duration_ms = 55.0;
  sample.max_drain_duration_ms = 50.0;
  return sample;
}
} // namespace

TEST(Bno086BacklogPolicy, healthyStreamDoesNotDetectBacklog)
{
  EXPECT_FALSE(IsBno086BacklogDetected(HealthySample(), Bno086BacklogDetectionConfig{}));
}

TEST(Bno086BacklogPolicy, lowRateGapsAndLargePacketsDetectBacklog)
{
  EXPECT_TRUE(IsBno086BacklogDetected(BacklogSample(), Bno086BacklogDetectionConfig{}));
}

TEST(Bno086BacklogPolicy, highPacketBytesAloneDoesNotDetectBacklog)
{
  Bno086BacklogSample sample = HealthySample();
  sample.full_packet_read_bytes_mean = 90.0;

  EXPECT_FALSE(IsBno086BacklogDetected(sample, Bno086BacklogDetectionConfig{}));
}

TEST(Bno086BacklogPolicy, startupGracePreventsAdaptiveEntry)
{
  Bno086AdaptiveRateLimitState state;
  Bno086AdaptiveRateLimitConfig config;
  config.backlog_windows_before_rate_limit = 2;
  config.startup_grace_sec = 10.0;

  UpdateBno086AdaptiveRateLimit(state, BacklogSample(), Bno086BacklogDetectionConfig{}, config,
                                1.0);
  const Bno086AdaptiveRateLimitDecision decision = UpdateBno086AdaptiveRateLimit(
      state, BacklogSample(), Bno086BacklogDetectionConfig{}, config, 2.0);

  EXPECT_FALSE(decision.enter_adaptive_mode);
  EXPECT_FALSE(state.active);
  EXPECT_EQ(state.consecutive_backlog_windows, 2U);
}

TEST(Bno086BacklogPolicy, consecutiveBacklogWindowsEnterAdaptiveMode)
{
  Bno086AdaptiveRateLimitState state;
  Bno086AdaptiveRateLimitConfig config;
  config.backlog_windows_before_rate_limit = 2;
  config.startup_grace_sec = 0.0;

  UpdateBno086AdaptiveRateLimit(state, BacklogSample(), Bno086BacklogDetectionConfig{}, config,
                                20.0);
  const Bno086AdaptiveRateLimitDecision decision = UpdateBno086AdaptiveRateLimit(
      state, BacklogSample(), Bno086BacklogDetectionConfig{}, config, 25.0);

  EXPECT_TRUE(decision.enter_adaptive_mode);
  EXPECT_TRUE(state.active);
  EXPECT_EQ(state.adaptive_rate_limit_entries, 1U);
}

TEST(Bno086BacklogPolicy, recoveryWindowsExitAdaptiveMode)
{
  Bno086AdaptiveRateLimitState state;
  Bno086AdaptiveRateLimitConfig config;
  config.backlog_windows_before_rate_limit = 1;
  config.recovery_windows_before_restore = 2;
  config.startup_grace_sec = 0.0;

  UpdateBno086AdaptiveRateLimit(state, BacklogSample(), Bno086BacklogDetectionConfig{}, config,
                                20.0);
  UpdateBno086AdaptiveRateLimit(state, HealthySample(), Bno086BacklogDetectionConfig{}, config,
                                25.0);
  const Bno086AdaptiveRateLimitDecision decision = UpdateBno086AdaptiveRateLimit(
      state, HealthySample(), Bno086BacklogDetectionConfig{}, config, 30.0);

  EXPECT_TRUE(decision.exit_adaptive_mode);
  EXPECT_FALSE(state.active);
  EXPECT_EQ(state.adaptive_rate_limit_exits, 1U);
}

TEST(Bno086BacklogPolicy, repeatedBacklogWindowsDoNotReenterWhenAlreadyActive)
{
  Bno086AdaptiveRateLimitState state;
  Bno086AdaptiveRateLimitConfig config;
  config.backlog_windows_before_rate_limit = 1;
  config.startup_grace_sec = 0.0;

  const Bno086AdaptiveRateLimitDecision firstDecision = UpdateBno086AdaptiveRateLimit(
      state, BacklogSample(), Bno086BacklogDetectionConfig{}, config, 20.0);
  const Bno086AdaptiveRateLimitDecision secondDecision = UpdateBno086AdaptiveRateLimit(
      state, BacklogSample(), Bno086BacklogDetectionConfig{}, config, 25.0);

  EXPECT_TRUE(firstDecision.enter_adaptive_mode);
  EXPECT_FALSE(secondDecision.enter_adaptive_mode);
  EXPECT_TRUE(state.active);
  EXPECT_EQ(state.adaptive_rate_limit_entries, 1U);
}
} // namespace OASIS::IMU::BNO086
