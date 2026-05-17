/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/core/Bno086SampleCoherence.hpp"

#include <gtest/gtest.h>

using namespace OASIS::IMU::BNO086;

TEST(Bno086SampleCoherence, SlightlyFutureOrientationIsAccepted)
{
  const SampleFreshnessResult result =
      EvaluateSampleFreshness(1'000'000'000, 1'009'000'000, 25'000'000, 10'000'000);

  EXPECT_EQ(result.status, SampleFreshnessStatus::Accepted);
  EXPECT_EQ(result.age_ns, -9'000'000);
}

TEST(Bno086SampleCoherence, FutureSampleBeyondOnePeriodIsRejected)
{
  const SampleFreshnessResult result =
      EvaluateSampleFreshness(1'000'000'000, 1'011'000'000, 25'000'000, 10'000'000);

  EXPECT_EQ(result.status, SampleFreshnessStatus::TooFuture);
}

TEST(Bno086SampleCoherence, OldSampleBeyondMaxAgeIsRejected)
{
  const SampleFreshnessResult result =
      EvaluateSampleFreshness(1'030'000'000, 1'000'000'000, 25'000'000, 10'000'000);

  EXPECT_EQ(result.status, SampleFreshnessStatus::TooOld);
}

TEST(Bno086SampleCoherence, EffectiveAgeAcceptsEightyMillisecondDefault)
{
  const int64_t maxAgeNs = EffectiveMaxPastAgeNs(25'000'000, 20'000'000, 80'000'000);
  const SampleFreshnessResult result =
      EvaluateSampleFreshness(1'080'000'000, 1'000'000'000, maxAgeNs, 10'000'000);

  EXPECT_EQ(maxAgeNs, 80'000'000);
  EXPECT_EQ(result.status, SampleFreshnessStatus::Accepted);
}

TEST(Bno086SampleCoherence, EffectiveAgeRejectsClearlyStaleSample)
{
  const int64_t maxAgeNs = EffectiveMaxPastAgeNs(25'000'000, 20'000'000, 80'000'000);
  const SampleFreshnessResult result =
      EvaluateSampleFreshness(1'101'000'000, 1'000'000'000, maxAgeNs, 10'000'000);

  EXPECT_EQ(result.status, SampleFreshnessStatus::TooOld);
}

TEST(Bno086SampleCoherence, CoreFrameWithinToleranceIsPublishable)
{
  EXPECT_TRUE(IsTimestampSpanCoherent(2'000'000'000, 2'015'000'000, 15'000'000));
}

TEST(Bno086SampleCoherence, CoreFrameSpanOfSeventyMillisecondsIsAccepted)
{
  const int64_t spanToleranceNs =
      EffectiveCoreSpanToleranceNs(20'000'000, 20'000'000, 20'000'000, 80'000'000);

  EXPECT_EQ(spanToleranceNs, 80'000'000);
  EXPECT_TRUE(IsTimestampSpanCoherent(2'000'000'000, 2'070'000'000, spanToleranceNs));
}

TEST(Bno086SampleCoherence, CoreFrameBeyondToleranceIsRejected)
{
  EXPECT_FALSE(IsTimestampSpanCoherent(2'000'000'000, 2'016'000'000, 15'000'000));
}

TEST(Bno086SampleCoherence, CoreFrameClearlyBeyondPolicyIsRejected)
{
  const int64_t spanToleranceNs =
      EffectiveCoreSpanToleranceNs(20'000'000, 20'000'000, 20'000'000, 80'000'000);

  EXPECT_FALSE(IsTimestampSpanCoherent(2'000'000'000, 2'101'000'000, spanToleranceNs));
}
