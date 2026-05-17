/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/Bno086ReportTimestampTracker.hpp"

#include <array>
#include <cstdint>
#include <optional>

#include <gtest/gtest.h>

using namespace OASIS::IMU::BNO086;

namespace
{
ReportTimestampTrackerInput Input(std::uint8_t sequence,
                                  std::optional<int64_t> expected_interval_ns,
                                  int64_t packet_host_stamp_ns,
                                  std::uint32_t delay_us = 0,
                                  std::optional<int64_t> shared_epoch_stamp_ns = std::nullopt)
{
  return ReportTimestampTrackerInput{
      sequence, expected_interval_ns,  packet_host_stamp_ns, delay_us > 0,
      delay_us, shared_epoch_stamp_ns,
  };
}
} // namespace

TEST(Bno086ReportTimestampTracker, firstAccelerometerSampleAnchorsToHost)
{
  Bno086ReportTimestampTracker tracker;

  const ReportTimestampTrackerResult result = tracker.Update(Input(1, 8'000'000, 1'000'000'000));

  EXPECT_EQ(result.stamp_ns, 1'000'000'000);
  EXPECT_TRUE(result.initialized);
  EXPECT_TRUE(result.reanchored);
  EXPECT_TRUE(result.used_host_anchor);
  EXPECT_FALSE(result.used_shared_epoch_anchor);
  EXPECT_FALSE(result.used_interval_cadence);
}

TEST(Bno086ReportTimestampTracker, firstKnownCadenceSampleUsesSharedEpochGrid)
{
  Bno086ReportTimestampTracker tracker;

  const ReportTimestampTrackerResult result =
      tracker.Update(Input(1, 10'000'000, 1'034'000'000, 0, 1'000'000'000));

  EXPECT_EQ(result.stamp_ns, 1'030'000'000);
  EXPECT_TRUE(result.initialized);
  EXPECT_TRUE(result.reanchored);
  EXPECT_TRUE(result.used_shared_epoch_anchor);
  EXPECT_FALSE(result.used_host_anchor);
}

TEST(Bno086ReportTimestampTracker, sharedEpochKeepsLateInitializingStreamsInPhase)
{
  Bno086ReportTimestampTracker accelTracker;
  Bno086ReportTimestampTracker gyroTracker;
  constexpr int64_t kSharedEpochNs = 2'000'000'000;

  const int64_t accelStampNs =
      accelTracker.Update(Input(1, 8'000'000, 2'064'000'000, 0, kSharedEpochNs)).stamp_ns;
  const int64_t gyroStampNs =
      gyroTracker.Update(Input(1, 10'000'000, 2'066'000'000, 0, kSharedEpochNs)).stamp_ns;

  EXPECT_EQ(accelStampNs, 2'064'000'000);
  EXPECT_EQ(gyroStampNs, 2'060'000'000);
  EXPECT_LE(accelStampNs - gyroStampNs, 10'000'000);
}

TEST(Bno086ReportTimestampTracker, accelerometerSequenceAdvancesByOne)
{
  Bno086ReportTimestampTracker tracker;

  ASSERT_EQ(tracker.Update(Input(1, 8'000'000, 1'000'000'000)).stamp_ns, 1'000'000'000);

  const ReportTimestampTrackerResult result = tracker.Update(Input(2, 8'000'000, 1'008'000'000));

  EXPECT_EQ(result.stamp_ns, 1'008'000'000);
  EXPECT_EQ(result.sequence_delta, 1);
  EXPECT_TRUE(result.used_interval_cadence);
  EXPECT_FALSE(result.gap_detected);
}

TEST(Bno086ReportTimestampTracker, accelerometerSequenceGapAdvancesByDelta)
{
  Bno086ReportTimestampTracker tracker;

  ASSERT_EQ(tracker.Update(Input(1, 8'000'000, 1'000'000'000)).stamp_ns, 1'000'000'000);

  const ReportTimestampTrackerResult result = tracker.Update(Input(4, 8'000'000, 1'024'000'000));

  EXPECT_EQ(result.stamp_ns, 1'024'000'000);
  EXPECT_EQ(result.sequence_delta, 3);
  EXPECT_TRUE(result.gap_detected);
  EXPECT_TRUE(result.used_interval_cadence);
}

TEST(Bno086ReportTimestampTracker, gyroAndRotationUseTenMillisecondCadence)
{
  Bno086ReportTimestampTracker gyroTracker;
  Bno086ReportTimestampTracker rotationTracker;

  ASSERT_EQ(gyroTracker.Update(Input(10, 10'000'000, 2'000'000'000)).stamp_ns, 2'000'000'000);
  ASSERT_EQ(rotationTracker.Update(Input(20, 10'000'000, 3'000'000'000)).stamp_ns, 3'000'000'000);

  EXPECT_EQ(gyroTracker.Update(Input(11, 10'000'000, 2'010'000'000)).stamp_ns, 2'010'000'000);
  EXPECT_EQ(rotationTracker.Update(Input(21, 10'000'000, 3'010'000'000)).stamp_ns, 3'010'000'000);
}

TEST(Bno086ReportTimestampTracker, linearAccelerationUsesTwentyMillisecondCadence)
{
  Bno086ReportTimestampTracker tracker;

  ASSERT_EQ(tracker.Update(Input(1, 20'000'000, 4'000'000'000)).stamp_ns, 4'000'000'000);

  EXPECT_EQ(tracker.Update(Input(2, 20'000'000, 4'020'000'000)).stamp_ns, 4'020'000'000);
}

TEST(Bno086ReportTimestampTracker, gravityUsesFortyMillisecondCadence)
{
  Bno086ReportTimestampTracker tracker;

  ASSERT_EQ(tracker.Update(Input(1, 40'000'000, 5'000'000'000)).stamp_ns, 5'000'000'000);

  EXPECT_EQ(tracker.Update(Input(2, 40'000'000, 5'040'000'000)).stamp_ns, 5'040'000'000);
}

TEST(Bno086ReportTimestampTracker, batchedAccelerometerSamplesUseCadenceWithStaleHost)
{
  Bno086ReportTimestampTracker tracker;

  constexpr int64_t kPacketHostStampNs = 6'000'000'000;

  ASSERT_EQ(tracker.Update(Input(1, 8'000'000, kPacketHostStampNs)).stamp_ns, kPacketHostStampNs);

  for (std::uint8_t sequence = 2; sequence <= 6; ++sequence)
  {
    const ReportTimestampTrackerResult result =
        tracker.Update(Input(sequence, 8'000'000, kPacketHostStampNs));
    const int64_t expectedStampNs =
        kPacketHostStampNs + (static_cast<int64_t>(sequence) - 1) * 8'000'000;
    EXPECT_EQ(result.stamp_ns, expectedStampNs);
    EXPECT_TRUE(result.used_interval_cadence);
  }
}

TEST(Bno086ReportTimestampTracker, accelerometerSequenceOneThroughTenKeepsEightMillisecondDeltas)
{
  Bno086ReportTimestampTracker tracker;

  constexpr int64_t kPacketHostStampNs = 10'000'000'000;
  int64_t previousStampNs = tracker.Update(Input(1, 8'000'000, kPacketHostStampNs)).stamp_ns;

  for (std::uint8_t sequence = 2; sequence <= 10; ++sequence)
  {
    const int64_t packetHostStampNs =
        kPacketHostStampNs + (static_cast<int64_t>(sequence) - 1) * 8'000'000;
    const ReportTimestampTrackerResult result =
        tracker.Update(Input(sequence, 8'000'000, packetHostStampNs));
    EXPECT_EQ(result.stamp_ns - previousStampNs, 8'000'000);
    EXPECT_TRUE(result.used_interval_cadence);
    EXPECT_FALSE(result.reanchored);
    previousStampNs = result.stamp_ns;
  }
}

TEST(Bno086ReportTimestampTracker, stalePacketHostDoesNotCollapseCadenceToOneNanosecond)
{
  Bno086ReportTimestampTracker tracker;

  constexpr int64_t kPacketHostStampNs = 11'000'000'000;

  ASSERT_EQ(tracker.Update(Input(1, 8'000'000, kPacketHostStampNs)).stamp_ns, kPacketHostStampNs);

  const ReportTimestampTrackerResult second =
      tracker.Update(Input(2, 8'000'000, kPacketHostStampNs));
  const ReportTimestampTrackerResult third =
      tracker.Update(Input(3, 8'000'000, kPacketHostStampNs));

  EXPECT_EQ(second.stamp_ns, kPacketHostStampNs + 8'000'000);
  EXPECT_EQ(third.stamp_ns, kPacketHostStampNs + 16'000'000);
  EXPECT_NE(second.stamp_ns, kPacketHostStampNs + 1);
  EXPECT_NE(third.stamp_ns, second.stamp_ns + 1);
}

TEST(Bno086ReportTimestampTracker, sequenceGapDeltaSixUsesCadenceWithoutReanchor)
{
  Bno086ReportTimestampTracker tracker;

  ASSERT_EQ(tracker.Update(Input(10, 8'000'000, 12'000'000'000)).stamp_ns, 12'000'000'000);

  const ReportTimestampTrackerResult result = tracker.Update(Input(16, 8'000'000, 12'048'000'000));

  EXPECT_EQ(result.sequence_delta, 6);
  EXPECT_EQ(result.stamp_ns, 12'048'000'000);
  EXPECT_TRUE(result.gap_detected);
  EXPECT_TRUE(result.used_interval_cadence);
  EXPECT_FALSE(result.reanchored);
}

TEST(Bno086ReportTimestampTracker, mixedCadencesPreserveExpectedDeltas)
{
  struct StreamCase
  {
    int64_t interval_ns;
  };

  const std::array<StreamCase, 5> streams{{
      {8'000'000},
      {10'000'000},
      {10'000'000},
      {20'000'000},
      {40'000'000},
  }};

  int64_t hostStampNs = 13'000'000'000;
  for (const StreamCase& stream : streams)
  {
    Bno086ReportTimestampTracker tracker;
    const int64_t firstStampNs = tracker.Update(Input(1, stream.interval_ns, hostStampNs)).stamp_ns;
    const int64_t secondStampNs =
        tracker.Update(Input(2, stream.interval_ns, hostStampNs)).stamp_ns;

    EXPECT_EQ(secondStampNs - firstStampNs, stream.interval_ns);
    hostStampNs += 1'000'000'000;
  }
}

TEST(Bno086ReportTimestampTracker, duplicateSequenceDoesNotCreateInterval)
{
  Bno086ReportTimestampTracker tracker;

  ASSERT_EQ(tracker.Update(Input(7, 8'000'000, 7'000'000'000)).stamp_ns, 7'000'000'000);

  const ReportTimestampTrackerResult duplicate = tracker.Update(Input(7, 8'000'000, 7'008'000'000));

  EXPECT_EQ(duplicate.stamp_ns, 7'000'000'000);
  EXPECT_TRUE(duplicate.duplicate_sequence);
  EXPECT_FALSE(duplicate.used_interval_cadence);
}

TEST(Bno086ReportTimestampTracker, largeSequenceJumpReanchorsAndMarksGap)
{
  Bno086ReportTimestampTracker tracker;

  ASSERT_EQ(tracker.Update(Input(1, 8'000'000, 8'000'000'000)).stamp_ns, 8'000'000'000);

  const ReportTimestampTrackerResult result = tracker.Update(Input(40, 8'000'000, 8'200'000'000));

  EXPECT_TRUE(result.gap_detected);
  EXPECT_TRUE(result.reanchored);
  EXPECT_TRUE(result.used_host_anchor);
  EXPECT_FALSE(result.used_interval_cadence);
  EXPECT_GE(result.stamp_ns, 8'150'000'000);
}

TEST(Bno086ReportTimestampTracker, BaseTimestampMovementCannotAffectCadence)
{
  Bno086ReportTimestampTracker tracker;

  ASSERT_EQ(tracker.Update(Input(1, 8'000'000, 9'000'000'000)).stamp_ns, 9'000'000'000);

  const ReportTimestampTrackerResult result = tracker.Update(Input(2, 8'000'000, 9'000'000'000));

  EXPECT_EQ(result.stamp_ns, 9'008'000'000);
  EXPECT_TRUE(result.used_interval_cadence);
  EXPECT_FALSE(result.reanchored);
}
