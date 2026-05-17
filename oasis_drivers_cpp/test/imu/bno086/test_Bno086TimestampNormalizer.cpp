/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/Bno086TimestampNormalizer.hpp"

#include <gtest/gtest.h>

using namespace OASIS::IMU::BNO086;

TEST(Bno086TimestampNormalizer, duplicateReconstructedStampUsesExpectedInterval)
{
  Bno086TimestampNormalizer normalizer;

  const TimestampNormalizationResult first = normalizer.Normalize(TimestampSample{1, 1'000, 1'000});
  EXPECT_EQ(first.stamp_ns, 1'000);
  EXPECT_FALSE(first.repaired_nonmonotonic);

  const TimestampNormalizationResult second =
      normalizer.Normalize(TimestampSample{2, 1'000, 8'001'000}, 8'000'000);
  EXPECT_EQ(second.stamp_ns, 8'001'000);
  EXPECT_TRUE(second.repaired_nonmonotonic);
  EXPECT_TRUE(second.repaired_duplicate_to_interval);
  EXPECT_FALSE(second.repaired_nonmonotonic_to_interval);
  EXPECT_FALSE(second.repaired_sequence_gap_to_interval);
  EXPECT_FALSE(second.interval_repair_clamped_to_host);
  EXPECT_FALSE(second.duplicate);
}

TEST(Bno086TimestampNormalizer, duplicateIntervalRepairClampsToHostWhenCandidateIsFuture)
{
  Bno086TimestampNormalizer normalizer;

  ASSERT_FALSE(normalizer.Normalize(TimestampSample{1, 1'000, 1'000}).duplicate);

  const TimestampNormalizationResult second =
      normalizer.Normalize(TimestampSample{2, 1'000, 5'000}, 8'000'000);
  EXPECT_EQ(second.stamp_ns, 5'000);
  EXPECT_TRUE(second.repaired_nonmonotonic);
  EXPECT_TRUE(second.repaired_duplicate_to_interval);
  EXPECT_TRUE(second.interval_repair_clamped_to_host);
  EXPECT_FALSE(second.interval_repair_bounded_to_legacy);
}

TEST(Bno086TimestampNormalizer, duplicateIntervalRepairFallsBackWhenHostCannotAdvance)
{
  Bno086TimestampNormalizer normalizer;

  ASSERT_FALSE(normalizer.Normalize(TimestampSample{1, 1'000, 1'000}).duplicate);

  const TimestampNormalizationResult second =
      normalizer.Normalize(TimestampSample{2, 1'000, 1'000}, 8'000'000);
  EXPECT_EQ(second.stamp_ns, 1'001);
  EXPECT_TRUE(second.repaired_nonmonotonic);
  EXPECT_TRUE(second.repaired_duplicate_to_interval);
  EXPECT_FALSE(second.interval_repair_clamped_to_host);
  EXPECT_TRUE(second.interval_repair_bounded_to_legacy);
}

TEST(Bno086TimestampNormalizer, futureIntervalCandidateAndStaleHostProducesLegacyPlusOne)
{
  Bno086TimestampNormalizer normalizer;

  ASSERT_FALSE(normalizer.Normalize(TimestampSample{4, 1'000'000, 1'000'000}).duplicate);

  const TimestampNormalizationResult repaired =
      normalizer.Normalize(TimestampSample{5, 1'000'000, 1'000'000}, 10'000'000);

  EXPECT_EQ(repaired.stamp_ns, 1'000'001);
  EXPECT_TRUE(repaired.repaired_nonmonotonic);
  EXPECT_TRUE(repaired.repaired_duplicate_to_interval);
  EXPECT_TRUE(repaired.interval_repair_bounded_to_legacy);
  EXPECT_FALSE(repaired.interval_repair_clamped_to_host);
}

TEST(Bno086TimestampNormalizer, futureIntervalCandidateAndAdvancingHostClampsToHost)
{
  Bno086TimestampNormalizer normalizer;

  ASSERT_FALSE(normalizer.Normalize(TimestampSample{4, 1'000'000, 1'000'000}).duplicate);

  const TimestampNormalizationResult repaired =
      normalizer.Normalize(TimestampSample{5, 1'000'000, 1'500'000}, 10'000'000);

  EXPECT_EQ(repaired.stamp_ns, 1'500'000);
  EXPECT_TRUE(repaired.repaired_nonmonotonic);
  EXPECT_TRUE(repaired.repaired_duplicate_to_interval);
  EXPECT_TRUE(repaired.interval_repair_clamped_to_host);
  EXPECT_FALSE(repaired.interval_repair_bounded_to_legacy);
}

TEST(Bno086TimestampNormalizer, duplicateStampWithAdvancedHostUsesExpectedInterval)
{
  Bno086TimestampNormalizer normalizer;

  ASSERT_FALSE(normalizer.Normalize(TimestampSample{4, 1'000'000, 1'000'000}).duplicate);

  const TimestampNormalizationResult repaired =
      normalizer.Normalize(TimestampSample{5, 1'000'000, 11'000'000}, 10'000'000);

  EXPECT_EQ(repaired.stamp_ns, 11'000'000);
  EXPECT_TRUE(repaired.repaired_nonmonotonic);
  EXPECT_TRUE(repaired.repaired_duplicate_to_interval);
  EXPECT_FALSE(repaired.interval_repair_clamped_to_host);
  EXPECT_FALSE(repaired.interval_repair_bounded_to_legacy);
}

TEST(Bno086TimestampNormalizer, nonmonotonicStampUsesExpectedInterval)
{
  Bno086TimestampNormalizer normalizer;

  ASSERT_FALSE(normalizer.Normalize(TimestampSample{10, 20'000'000, 20'000'000}).duplicate);

  const TimestampNormalizationResult repaired =
      normalizer.Normalize(TimestampSample{11, 19'000'000, 30'000'000}, 10'000'000);
  EXPECT_EQ(repaired.stamp_ns, 30'000'000);
  EXPECT_TRUE(repaired.repaired_nonmonotonic);
  EXPECT_FALSE(repaired.repaired_duplicate_to_interval);
  EXPECT_TRUE(repaired.repaired_nonmonotonic_to_interval);
  EXPECT_FALSE(repaired.repaired_sequence_gap_to_interval);
}

TEST(Bno086TimestampNormalizer, sequenceGapUsesExpectedIntervalMultiplier)
{
  Bno086TimestampNormalizer normalizer;

  ASSERT_FALSE(normalizer.Normalize(TimestampSample{10, 20'000'000, 20'000'000}).duplicate);

  const TimestampNormalizationResult repaired =
      normalizer.Normalize(TimestampSample{13, 20'000'000, 44'000'000}, 8'000'000);
  EXPECT_EQ(repaired.stamp_ns, 44'000'000);
  EXPECT_TRUE(repaired.sequence_gap);
  EXPECT_FALSE(repaired.repaired_duplicate_to_interval);
  EXPECT_TRUE(repaired.repaired_sequence_gap_to_interval);
  EXPECT_FALSE(repaired.interval_repair_clamped_to_host);
}

TEST(Bno086TimestampNormalizer, sequenceGapRepairClampsToHost)
{
  Bno086TimestampNormalizer normalizer;

  ASSERT_FALSE(normalizer.Normalize(TimestampSample{10, 20'000'000, 20'000'000}).duplicate);

  const TimestampNormalizationResult repaired =
      normalizer.Normalize(TimestampSample{13, 20'000'000, 30'000'000}, 8'000'000);
  EXPECT_EQ(repaired.stamp_ns, 30'000'000);
  EXPECT_TRUE(repaired.sequence_gap);
  EXPECT_TRUE(repaired.repaired_sequence_gap_to_interval);
  EXPECT_TRUE(repaired.interval_repair_clamped_to_host);
  EXPECT_LE(repaired.stamp_ns, 30'000'000);
}

TEST(Bno086TimestampNormalizer, sequenceWrapUsesExpectedInterval)
{
  Bno086TimestampNormalizer normalizer;

  ASSERT_FALSE(normalizer.Normalize(TimestampSample{255, 20'000'000, 20'000'000}).duplicate);

  const TimestampNormalizationResult repaired =
      normalizer.Normalize(TimestampSample{0, 20'000'000, 28'000'000}, 8'000'000);
  EXPECT_EQ(repaired.stamp_ns, 28'000'000);
  EXPECT_FALSE(repaired.sequence_gap);
  EXPECT_TRUE(repaired.repaired_duplicate_to_interval);
  EXPECT_FALSE(repaired.repaired_sequence_gap_to_interval);
}

TEST(Bno086TimestampNormalizer, linearAccelerationIntervalRepairWorksWhenHostAdvanced)
{
  Bno086TimestampNormalizer normalizer;

  ASSERT_FALSE(normalizer.Normalize(TimestampSample{40, 100'000'000, 100'000'000}).duplicate);

  const TimestampNormalizationResult repaired =
      normalizer.Normalize(TimestampSample{41, 100'000'000, 120'000'000}, 20'000'000);
  EXPECT_EQ(repaired.stamp_ns, 120'000'000);
  EXPECT_TRUE(repaired.repaired_duplicate_to_interval);
  EXPECT_FALSE(repaired.interval_repair_clamped_to_host);
}

TEST(Bno086TimestampNormalizer, newSequenceNonAdvancingStampUsesLegacyRepairWithoutInterval)
{
  Bno086TimestampNormalizer normalizer;

  ASSERT_FALSE(normalizer.Normalize(TimestampSample{1, 1'000, 1'000}).duplicate);

  const TimestampNormalizationResult second =
      normalizer.Normalize(TimestampSample{2, 1'000, 1'000});
  EXPECT_EQ(second.stamp_ns, 1'001);
  EXPECT_TRUE(second.repaired_nonmonotonic);
  EXPECT_FALSE(second.repaired_duplicate_to_interval);
  EXPECT_FALSE(second.duplicate);
}

TEST(Bno086TimestampNormalizer, sameSequenceAndStampIsDuplicate)
{
  Bno086TimestampNormalizer normalizer;

  ASSERT_FALSE(normalizer.Normalize(TimestampSample{7, 2'000, 2'000}).duplicate);

  const TimestampNormalizationResult duplicate =
      normalizer.Normalize(TimestampSample{7, 2'000, 2'000}, 8'000'000);
  EXPECT_TRUE(duplicate.duplicate);
  EXPECT_FALSE(duplicate.repaired_nonmonotonic);
}

TEST(Bno086TimestampNormalizer, implausibleReconstructionUsesHostPacketTime)
{
  Bno086TimestampNormalizer normalizer;

  const TimestampNormalizationResult reset =
      normalizer.Normalize(TimestampSample{1, 1'000'000'000, 1'101'000'001});
  EXPECT_EQ(reset.stamp_ns, 1'101'000'001);
  EXPECT_TRUE(reset.reconstruction_reset);
}
