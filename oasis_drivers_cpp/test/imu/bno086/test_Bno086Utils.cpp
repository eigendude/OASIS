/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/utils/Bno086ReportUtils.hpp"
#include "imu/bno086/utils/Bno086TimingUtils.hpp"

#include <cstdint> // NOLINT(build/include_order)

#include <gtest/gtest.h> // NOLINT(build/include_order)

using OASIS::IMU::BNO086::DurationNsFromUs;
using OASIS::IMU::BNO086::RateHzToIntervalUs;
using OASIS::IMU::BNO086::ReportId;
using OASIS::IMU::BNO086::ReportName;

TEST(Bno086ReportUtils, ReportNameReturnsKnownNames)
{
  EXPECT_STREQ("accelerometer", ReportName(ReportId::Accelerometer));
  EXPECT_STREQ("gyro", ReportName(ReportId::GyroscopeCalibrated));
  EXPECT_STREQ("linear_acceleration", ReportName(ReportId::LinearAcceleration));
  EXPECT_STREQ("rotation_vector", ReportName(ReportId::RotationVector));
  EXPECT_STREQ("gravity", ReportName(ReportId::Gravity));
}

TEST(Bno086ReportUtils, ReportNameReturnsUnknownForUnknownReport)
{
  EXPECT_STREQ("unknown", ReportName(static_cast<ReportId>(0xff)));
}

TEST(Bno086TimingUtils, RateHzToIntervalUsConvertsPositiveRates)
{
  EXPECT_EQ(20'000U, RateHzToIntervalUs(50.0).value());
  EXPECT_EQ(1U, RateHzToIntervalUs(2'000'000.0).value());
}

TEST(Bno086TimingUtils, RateHzToIntervalUsRejectsNonPositiveRates)
{
  EXPECT_FALSE(RateHzToIntervalUs(0.0).has_value());
  EXPECT_FALSE(RateHzToIntervalUs(-1.0).has_value());
}

TEST(Bno086TimingUtils, DurationNsFromUsConvertsMicrosecondsToNanoseconds)
{
  EXPECT_EQ(static_cast<int64_t>(12'345'000), DurationNsFromUs(12'345));
}
