/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "power_meter/SampleMovingAverage.hpp"

#include <gtest/gtest.h>

using OASIS::PowerMeter::ActivePowerInvariantMonitor;
using OASIS::PowerMeter::Sample;
using OASIS::PowerMeter::SampleMovingAverage;

namespace
{
Sample MakeSample(double voltage, double current, double power)
{
  Sample sample;
  sample.voltage = voltage;
  sample.current = current;
  sample.power = power;
  return sample;
}
} // namespace

TEST(SampleMovingAverage, LengthOneBypassesSmoothing)
{
  SampleMovingAverage filter(1);
  EXPECT_DOUBLE_EQ(filter.Update(MakeSample(1.0, 2.0, -3.0)).power, -3.0);
  const Sample result = filter.Update(MakeSample(4.0, 5.0, -6.0));
  EXPECT_DOUBLE_EQ(result.voltage, 4.0);
  EXPECT_DOUBLE_EQ(result.current, 5.0);
  EXPECT_DOUBLE_EQ(result.power, -6.0);
}

TEST(SampleMovingAverage, LengthThreeAveragesSignedComponents)
{
  SampleMovingAverage filter(3);
  filter.Update(MakeSample(1.0, 2.0, -3.0));
  filter.Update(MakeSample(2.0, 4.0, 3.0));
  const Sample result = filter.Update(MakeSample(3.0, 6.0, -6.0));
  EXPECT_DOUBLE_EQ(result.voltage, 2.0);
  EXPECT_DOUBLE_EQ(result.current, 4.0);
  EXPECT_DOUBLE_EQ(result.power, -2.0);
}

TEST(SampleMovingAverage, ResetDropsPreFailureHistory)
{
  SampleMovingAverage filter(3);
  filter.Update(MakeSample(100.0, 100.0, 100.0));
  filter.Reset();
  const Sample result = filter.Update(MakeSample(1.0, 2.0, -3.0));
  EXPECT_DOUBLE_EQ(result.voltage, 1.0);
  EXPECT_DOUBLE_EQ(result.current, 2.0);
  EXPECT_DOUBLE_EQ(result.power, -3.0);
}

TEST(ActivePowerInvariantMonitor, CountsOnlyRawViolationsBeforeFiltering)
{
  ActivePowerInvariantMonitor monitor;
  SampleMovingAverage filter(3);

  const Sample first = MakeSample(0.1, 0.1, 0.01);
  EXPECT_FALSE(monitor.Evaluate(first, 0.1));
  filter.Update(first);

  const Sample second = MakeSample(10.0, 10.0, 100.0);
  EXPECT_FALSE(monitor.Evaluate(second, 0.1));
  const Sample filtered = filter.Update(second);

  // Mixing components can make a filtered tuple fail the bound even though
  // the current raw sample is valid; filtered values must never be evaluated
  EXPECT_GT(std::abs(filtered.power), filtered.voltage * filtered.current + 0.1);
  EXPECT_EQ(monitor.GetViolationCount(), 0U);

  EXPECT_TRUE(monitor.Evaluate(MakeSample(1.0, 1.0, 2.0), 0.1));
  EXPECT_EQ(monitor.GetViolationCount(), 1U);
}
