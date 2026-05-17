/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/utils/Bno086CovarianceUtils.hpp"
#include "imu/bno086/utils/Bno086MathUtils.hpp"
#include "imu/bno086/utils/Bno086ReportUtils.hpp"
#include "imu/bno086/utils/Bno086TimingUtils.hpp"

#include <array>
#include <cstdint> // NOLINT(build/include_order)

#include <gtest/gtest.h> // NOLINT(build/include_order)

using OASIS::IMU::BNO086::CovarianceFromAccuracyBucket;
using OASIS::IMU::BNO086::DurationNsFromUs;
using OASIS::IMU::BNO086::NormalizeQuaternion;
using OASIS::IMU::BNO086::QToDouble;
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

TEST(Bno086MathUtils, QToDoubleConvertsFixedPointValues)
{
  EXPECT_DOUBLE_EQ(1.0, QToDouble(256, 8));
  EXPECT_DOUBLE_EQ(-0.5, QToDouble(-128, 8));
}

TEST(Bno086MathUtils, NormalizeQuaternionScalesToUnitLength)
{
  std::array<double, 4> quaternion{0.0, 0.0, 0.0, 2.0};

  NormalizeQuaternion(quaternion);

  EXPECT_DOUBLE_EQ(0.0, quaternion[0]);
  EXPECT_DOUBLE_EQ(0.0, quaternion[1]);
  EXPECT_DOUBLE_EQ(0.0, quaternion[2]);
  EXPECT_DOUBLE_EQ(1.0, quaternion[3]);
}

TEST(Bno086CovarianceUtils, CovarianceFromAccuracyBucketUsesExpectedDiagonal)
{
  const OASIS::IMU::Mat3 covariance = CovarianceFromAccuracyBucket(2, 4.0, 2.0, 0.8, 0.25);

  EXPECT_DOUBLE_EQ(0.64, covariance[0][0]);
  EXPECT_DOUBLE_EQ(0.64, covariance[1][1]);
  EXPECT_DOUBLE_EQ(0.64, covariance[2][2]);
  EXPECT_DOUBLE_EQ(0.0, covariance[0][1]);
}
