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

TEST(Bno086TimestampNormalizer, newSequenceNonAdvancingStampGetsOneNanosecondRepair)
{
  Bno086TimestampNormalizer normalizer;

  const TimestampNormalizationResult first = normalizer.Normalize(TimestampSample{1, 1'000, 1'000});
  EXPECT_EQ(first.stamp_ns, 1'000);
  EXPECT_FALSE(first.repaired_nonmonotonic);

  const TimestampNormalizationResult second =
      normalizer.Normalize(TimestampSample{2, 1'000, 1'000});
  EXPECT_EQ(second.stamp_ns, 1'001);
  EXPECT_TRUE(second.repaired_nonmonotonic);
  EXPECT_FALSE(second.duplicate);
}

TEST(Bno086TimestampNormalizer, sameSequenceAndStampIsDuplicate)
{
  Bno086TimestampNormalizer normalizer;

  ASSERT_FALSE(normalizer.Normalize(TimestampSample{7, 2'000, 2'000}).duplicate);

  const TimestampNormalizationResult duplicate =
      normalizer.Normalize(TimestampSample{7, 2'000, 2'000});
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
