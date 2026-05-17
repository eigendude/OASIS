/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/Bno086ReportTimestampTracker.hpp"

#include <cstdint>
#include <optional>

#include <gtest/gtest.h>

using namespace OASIS::IMU::BNO086;

namespace
{
ReportTimestampTrackerInput Input(std::uint8_t sequence,
                                  std::optional<int64_t> expected_interval_ns,
                                  int64_t packet_host_stamp_ns,
                                  std::uint32_t delay_us = 0)
{
  return ReportTimestampTrackerInput{
      sequence, expected_interval_ns, packet_host_stamp_ns, delay_us > 0, delay_us,
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
  EXPECT_FALSE(result.used_interval_cadence);
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
