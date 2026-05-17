/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/core/Bno086ImuGravityAccelHistory.hpp"

#include <cstdint>

#include <gtest/gtest.h>

using namespace OASIS::IMU::BNO086;

namespace
{
Bno086ImuGravityAccelSample Sample(int64_t stamp_ns, double x)
{
  Bno086ImuGravityAccelSample sample;
  sample.has_sample = true;
  sample.stamp_ns = stamp_ns;
  sample.accel_mps2 = OASIS::IMU::Vec3{x, 0.0, -9.81};
  return sample;
}
} // namespace

TEST(Bno086ImuGravityAccelHistory, selectsNearestPastSample)
{
  Bno086ImuGravityAccelHistory history;
  history.Push(Sample(100, 1.0));
  history.Push(Sample(300, 3.0));
  history.Push(Sample(200, 2.0));

  const std::optional<Bno086ImuGravityAccelSample> selected = history.SelectAtOrBefore(250, 1000);

  ASSERT_TRUE(selected.has_value());
  EXPECT_EQ(selected->stamp_ns, 200);
  EXPECT_DOUBLE_EQ(selected->accel_mps2[0], 2.0);
}

TEST(Bno086ImuGravityAccelHistory, selectsFutureWithinToleranceWhenNoPastExists)
{
  Bno086ImuGravityAccelHistory history;
  history.Push(Sample(120, 1.0));
  history.Push(Sample(140, 2.0));

  const std::optional<Bno086ImuGravityAccelSample> selected = history.SelectAtOrBefore(100, 25);

  ASSERT_TRUE(selected.has_value());
  EXPECT_EQ(selected->stamp_ns, 120);
}

TEST(Bno086ImuGravityAccelHistory, rejectsFarFutureSample)
{
  Bno086ImuGravityAccelHistory history;
  history.Push(Sample(140, 1.0));

  EXPECT_FALSE(history.SelectAtOrBefore(100, 25).has_value());
}

TEST(Bno086ImuGravityAccelHistory, resetClearsHistory)
{
  Bno086ImuGravityAccelHistory history;
  history.Push(Sample(100, 1.0));
  history.Reset();

  EXPECT_FALSE(history.SelectAtOrBefore(200, 1000).has_value());
}

TEST(Bno086ImuGravityAccelHistory, fixedHistoryRetainsMostRecentWindow)
{
  Bno086ImuGravityAccelHistory history;
  for (int index = 0; index < 1100; ++index)
    history.Push(Sample(index * 10, static_cast<double>(index)));

  const std::optional<Bno086ImuGravityAccelSample> oldSelection = history.SelectAtOrBefore(500, 0);
  const std::optional<Bno086ImuGravityAccelSample> recentSelection =
      history.SelectAtOrBefore(10'990, 0);

  EXPECT_FALSE(oldSelection.has_value());
  ASSERT_TRUE(recentSelection.has_value());
  EXPECT_EQ(recentSelection->stamp_ns, 10'990);
}
