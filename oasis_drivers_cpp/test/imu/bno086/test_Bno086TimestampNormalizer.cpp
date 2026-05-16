/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/Bno086TimestampNormalizer.hpp"

#include <cstdint>

#include <gtest/gtest.h>

namespace OASIS::IMU::BNO086
{
namespace
{
TimestampNormalizationConfig TestConfig()
{
  TimestampNormalizationConfig config;
  config.expected_interval_us = 10'000;
  return config;
}
} // namespace

TEST(Bno086TimestampNormalizer, firstAccelerometerSampleAcceptsRawStamp)
{
  ReportTimestampState state;

  const TimestampNormalizationResult result =
      NormalizeReportTimestamp(state, 7, 1'000'000'000, TestConfig());

  EXPECT_EQ(result.status, TimestampNormalizationStatus::FirstSample);
  EXPECT_EQ(result.stamp_ns, 1'000'000'000);
  EXPECT_TRUE(state.has_sample);
  EXPECT_EQ(state.last_sequence, 7);
  EXPECT_EQ(state.last_stamp_ns, 1'000'000'000);
}

TEST(Bno086TimestampNormalizer, sequenceAdvancesAndRawStampAdvancesWithoutRepair)
{
  ReportTimestampState state;
  NormalizeReportTimestamp(state, 7, 1'000'000'000, TestConfig());

  const TimestampNormalizationResult result =
      NormalizeReportTimestamp(state, 8, 1'010'000'000, TestConfig());

  EXPECT_EQ(result.status, TimestampNormalizationStatus::Accepted);
  EXPECT_EQ(result.sequence_delta, 1);
  EXPECT_EQ(result.raw_stamp_delta_ns, 10'000'000);
  EXPECT_EQ(result.normalized_stamp_delta_ns, 10'000'000);
  EXPECT_EQ(result.stamp_ns, 1'010'000'000);
  EXPECT_EQ(state.repaired_nonmonotonic, 0U);
}

TEST(Bno086TimestampNormalizer, sequenceAdvancesAndEqualRawStampIsRepaired)
{
  ReportTimestampState state;
  NormalizeReportTimestamp(state, 7, 1'000'000'000, TestConfig());

  const TimestampNormalizationResult result =
      NormalizeReportTimestamp(state, 8, 1'000'000'000, TestConfig());

  EXPECT_EQ(result.status, TimestampNormalizationStatus::RepairedNonmonotonic);
  EXPECT_EQ(result.sequence_delta, 1);
  EXPECT_EQ(result.raw_stamp_delta_ns, 0);
  EXPECT_EQ(result.normalized_stamp_delta_ns, 1);
  EXPECT_EQ(result.stamp_ns, 1'000'000'001);
  EXPECT_EQ(state.repaired_nonmonotonic, 1U);
}

TEST(Bno086TimestampNormalizer, sequenceAdvancesAndBackwardRawStampIsRepaired)
{
  ReportTimestampState state;
  NormalizeReportTimestamp(state, 7, 1'000'000'000, TestConfig());

  const TimestampNormalizationResult result =
      NormalizeReportTimestamp(state, 8, 999'000'000, TestConfig());

  EXPECT_EQ(result.status, TimestampNormalizationStatus::RepairedNonmonotonic);
  EXPECT_EQ(result.raw_stamp_delta_ns, -1'000'000);
  EXPECT_GT(result.stamp_ns, 1'000'000'000);
  EXPECT_EQ(result.stamp_ns, 1'000'000'001);
}

TEST(Bno086TimestampNormalizer, sameSequenceAndNonAdvancingStampIsTrueDuplicate)
{
  ReportTimestampState state;
  NormalizeReportTimestamp(state, 7, 1'000'000'000, TestConfig());

  const TimestampNormalizationResult result =
      NormalizeReportTimestamp(state, 7, 1'000'000'000, TestConfig());

  EXPECT_EQ(result.status, TimestampNormalizationStatus::TrueDuplicate);
  EXPECT_EQ(result.sequence_delta, 0);
  EXPECT_EQ(result.stamp_ns, 1'000'000'000);
  EXPECT_EQ(result.normalized_stamp_delta_ns, 0);
  EXPECT_EQ(state.true_duplicate, 1U);
}

TEST(Bno086TimestampNormalizer, uint8WraparoundFrom255To0GivesSequenceDeltaOne)
{
  ReportTimestampState state;
  NormalizeReportTimestamp(state, 255, 1'000'000'000, TestConfig());

  const TimestampNormalizationResult result =
      NormalizeReportTimestamp(state, 0, 1'010'000'000, TestConfig());

  EXPECT_EQ(result.status, TimestampNormalizationStatus::Accepted);
  EXPECT_EQ(result.sequence_delta, 1);
  EXPECT_FALSE(result.sequence_gap);
}

TEST(Bno086TimestampNormalizer, largeSequenceGapIsDetectedAndCounted)
{
  ReportTimestampState state;
  NormalizeReportTimestamp(state, 10, 1'000'000'000, TestConfig());

  const TimestampNormalizationResult result =
      NormalizeReportTimestamp(state, 14, 1'040'000'000, TestConfig());

  EXPECT_EQ(result.status, TimestampNormalizationStatus::Accepted);
  EXPECT_EQ(result.sequence_delta, 4);
  EXPECT_TRUE(result.sequence_gap);
  EXPECT_EQ(state.sequence_gap_count, 1U);
  EXPECT_EQ(state.sequence_gap_max, 4);
}

TEST(Bno086TimestampNormalizer, repeatedRawStampWithNewSequenceGetsDistinctSignature)
{
  ReportTimestampState state;
  const TimestampNormalizationResult first =
      NormalizeReportTimestamp(state, 31, 1'000'000'000, TestConfig());

  const TimestampNormalizationResult second =
      NormalizeReportTimestamp(state, 32, 1'000'000'000, TestConfig());

  EXPECT_EQ(second.status, TimestampNormalizationStatus::RepairedNonmonotonic);
  EXPECT_NE(second.sequence_delta, 0);
  EXPECT_GT(second.stamp_ns, first.stamp_ns);
  EXPECT_NE(second.stamp_ns, first.stamp_ns);
  EXPECT_EQ(state.true_duplicate, 0U);
}
} // namespace OASIS::IMU::BNO086
