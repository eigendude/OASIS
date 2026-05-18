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
  EXPECT_FALSE(duplicate.reanchored_to_host);
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
  EXPECT_FALSE(result.reanchored_to_host);
}

TEST(Bno086TimestampCadence, lateStartingReportStreamStartsNearPacketHostTime)
{
  Bno086TimestampCadence cadence;
  ASSERT_EQ(*cadence.Finalize(Event(ReportId::Accelerometer, 1), 1'000'000'000, 8'000'000).stamp_ns,
            1'000'000'000);

  const Bno086TimestampCadenceResult gravity =
      cadence.Finalize(Event(ReportId::Gravity, 1), 69'003'000'000, 40'000'000);

  ASSERT_TRUE(gravity.stamp_ns.has_value());
  EXPECT_EQ(*gravity.stamp_ns, 69'000'000'000);
  EXPECT_FALSE(gravity.reanchored_to_host);
  EXPECT_LE(69'003'000'000 - *gravity.stamp_ns, 40'000'000);
}

TEST(Bno086TimestampCadence, initializedStreamReanchorsWhenCadenceIsSixtySecondsBehindHost)
{
  Bno086TimestampCadence cadence;
  ASSERT_EQ(
      *cadence.Finalize(Event(ReportId::LinearAcceleration, 1), 1'000'000'000, 20'000'000).stamp_ns,
      1'000'000'000);

  const Bno086TimestampCadenceResult result =
      cadence.Finalize(Event(ReportId::LinearAcceleration, 2), 61'020'000'000, 20'000'000);

  ASSERT_TRUE(result.stamp_ns.has_value());
  EXPECT_EQ(result.candidate_stamp_ns, 1'020'000'000);
  EXPECT_EQ(result.host_anchor_stamp_ns, 61'020'000'000);
  EXPECT_EQ(result.reanchor_delta_ns, -60'000'000'000);
  EXPECT_EQ(*result.stamp_ns, 61'020'000'000);
  EXPECT_TRUE(result.reanchored_to_host);
}

TEST(Bno086TimestampCadence, futureSkewedCadenceCandidateReanchorsToHost)
{
  Bno086TimestampCadence cadence;
  ASSERT_EQ(*cadence.Finalize(Event(ReportId::GyroscopeCalibrated, 1), 10'000'000'000, 10'000'000)
                 .stamp_ns,
            10'000'000'000);

  const Bno086TimestampCadenceResult result =
      cadence.Finalize(Event(ReportId::GyroscopeCalibrated, 201), 10'010'000'000, 10'000'000);

  ASSERT_TRUE(result.stamp_ns.has_value());
  EXPECT_EQ(result.candidate_stamp_ns, 12'000'000'000);
  EXPECT_EQ(result.host_anchor_stamp_ns, 10'010'000'000);
  EXPECT_EQ(result.reanchor_delta_ns, 1'990'000'000);
  EXPECT_EQ(*result.stamp_ns, 10'010'000'000);
  EXPECT_TRUE(result.reanchored_to_host);
}

TEST(Bno086TimestampCadence, reanchoredOutputRemainsMonotonic)
{
  Bno086TimestampCadence cadence;

  const Bno086TimestampCadenceResult first =
      cadence.Finalize(Event(ReportId::RotationVector, 1), 20'000'000'000, 10'000'000);
  const Bno086TimestampCadenceResult reanchored =
      cadence.Finalize(Event(ReportId::RotationVector, 2), 80'010'000'000, 10'000'000);
  const Bno086TimestampCadenceResult next =
      cadence.Finalize(Event(ReportId::RotationVector, 3), 80'020'000'000, 10'000'000);

  ASSERT_TRUE(first.stamp_ns.has_value());
  ASSERT_TRUE(reanchored.stamp_ns.has_value());
  ASSERT_TRUE(next.stamp_ns.has_value());
  EXPECT_LT(*first.stamp_ns, *reanchored.stamp_ns);
  EXPECT_LT(*reanchored.stamp_ns, *next.stamp_ns);
  EXPECT_TRUE(reanchored.reanchored_to_host);
  EXPECT_EQ(*next.stamp_ns, 80'020'000'000);
}

TEST(Bno086TimestampCadence, hostReanchorIsNotOverwrittenByOuterMonotonicGuard)
{
  Bno086TimestampCadence cadence;

  ASSERT_EQ(*cadence.Finalize(Event(ReportId::Gravity, 1), 100'000'000'000, 40'000'000).stamp_ns,
            100'000'000'000);
  ASSERT_EQ(*cadence.Finalize(Event(ReportId::Gravity, 2), 100'040'000'000, 40'000'000).stamp_ns,
            100'040'000'000);

  const Bno086TimestampCadenceResult reanchored =
      cadence.Finalize(Event(ReportId::Gravity, 3), 99'500'000'000, 40'000'000);
  const Bno086TimestampCadenceResult next =
      cadence.Finalize(Event(ReportId::Gravity, 4), 99'540'000'000, 40'000'000);

  ASSERT_TRUE(reanchored.stamp_ns.has_value());
  ASSERT_TRUE(next.stamp_ns.has_value());
  EXPECT_EQ(reanchored.candidate_stamp_ns, 100'080'000'000);
  EXPECT_EQ(reanchored.host_anchor_stamp_ns, 99'500'000'000);
  EXPECT_EQ(reanchored.reanchor_delta_ns, 580'000'000);
  EXPECT_EQ(*reanchored.stamp_ns, 99'500'000'000);
  EXPECT_TRUE(reanchored.reanchored_to_host);
  EXPECT_FALSE(reanchored.monotonic_guard_adjusted);
  EXPECT_EQ(*next.stamp_ns, 99'540'000'000);
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
