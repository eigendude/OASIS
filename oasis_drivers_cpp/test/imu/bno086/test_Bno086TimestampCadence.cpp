/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/core/Bno086TimestampCadence.hpp"

#include <optional>

#include <gtest/gtest.h>

using namespace OASIS::IMU::BNO086;

namespace
{
SensorEvent Event(ReportId report_id, std::uint8_t sequence)
{
  SensorEvent event;
  event.report_id = report_id;
  event.sequence = sequence;
  return event;
}
} // namespace

TEST(Bno086TimestampCadence, emitsCadenceTimestamps)
{
  Bno086TimestampCadence cadence;

  const Bno086TimestampCadenceResult first =
      cadence.Finalize(Event(ReportId::Accelerometer, 10), 1'000'000'000, 8'000'000);
  const Bno086TimestampCadenceResult second =
      cadence.Finalize(Event(ReportId::Accelerometer, 11), 1'008'000'000, 8'000'000);

  ASSERT_TRUE(first.stamp_ns.has_value());
  ASSERT_TRUE(second.stamp_ns.has_value());
  EXPECT_EQ(*first.stamp_ns, 1'000'000'000);
  EXPECT_EQ(*second.stamp_ns, 1'008'000'000);
}

TEST(Bno086TimestampCadence, duplicateSequenceReturnsNoStamp)
{
  Bno086TimestampCadence cadence;
  ASSERT_TRUE(cadence.Finalize(Event(ReportId::GyroscopeCalibrated, 3), 2'000'000'000, 10'000'000)
                  .stamp_ns.has_value());

  const Bno086TimestampCadenceResult duplicate =
      cadence.Finalize(Event(ReportId::GyroscopeCalibrated, 3), 2'010'000'000, 10'000'000);

  EXPECT_FALSE(duplicate.stamp_ns.has_value());
  EXPECT_TRUE(duplicate.duplicate_sequence);
}

TEST(Bno086TimestampCadence, sequenceGapAdvancesByInterval)
{
  Bno086TimestampCadence cadence;
  ASSERT_EQ(
      *cadence.Finalize(Event(ReportId::RotationVector, 1), 3'000'000'000, 10'000'000).stamp_ns,
      3'000'000'000);

  const Bno086TimestampCadenceResult result =
      cadence.Finalize(Event(ReportId::RotationVector, 4), 3'010'000'000, 10'000'000);

  ASSERT_TRUE(result.stamp_ns.has_value());
  EXPECT_EQ(*result.stamp_ns, 3'030'000'000);
  EXPECT_EQ(result.sequence_delta, 3);
}

TEST(Bno086TimestampCadence, monotonicGuardAdjustsStaleHostTimestampWithoutInterval)
{
  Bno086TimestampCadence cadence;
  ASSERT_EQ(*cadence.Finalize(Event(ReportId::Gravity, 1), 4'000'000'000, std::nullopt).stamp_ns,
            4'000'000'000);

  const Bno086TimestampCadenceResult result =
      cadence.Finalize(Event(ReportId::Gravity, 2), 3'999'999'000, std::nullopt);

  ASSERT_TRUE(result.stamp_ns.has_value());
  EXPECT_EQ(*result.stamp_ns, 4'000'000'001);
  EXPECT_TRUE(result.monotonic_guard_adjusted);
  ASSERT_TRUE(result.last_stamp_ns.has_value());
  EXPECT_EQ(*result.last_stamp_ns, 4'000'000'000);
}

TEST(Bno086TimestampCadence, outOfRangeReportIdDoesNotIndexOutOfRange)
{
  Bno086TimestampCadence cadence;
  SensorEvent event = Event(static_cast<ReportId>(0xff), 1);

  const Bno086TimestampCadenceResult result = cadence.Finalize(event, 5'000'000'000, 10'000'000);

  EXPECT_FALSE(result.stamp_ns.has_value());
  EXPECT_FALSE(result.monotonic_guard_adjusted);
}
