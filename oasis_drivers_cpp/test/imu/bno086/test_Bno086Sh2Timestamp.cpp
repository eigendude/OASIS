/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/core/Bno086Sh2Timestamp.hpp"

#include <gtest/gtest.h>

using namespace OASIS::IMU::BNO086;

TEST(Bno086Sh2Timestamp, timebaseDeltaWithoutReportDelaySubtractsFromInterrupt)
{
  const Bno086Sh2TimestampResult result =
      ComputeBno086Sh2Timestamp(Bno086Sh2TimestampInput{1'000'000'000, true, 120, true, 0});

  EXPECT_EQ(result.stamp_ns, 988'000'000);
  EXPECT_TRUE(result.used_timebase_reference);
  EXPECT_TRUE(result.used_report_delay);
  EXPECT_FALSE(result.used_missing_timebase_fallback);
}

TEST(Bno086Sh2Timestamp, reportDelayOffsetsTimebase)
{
  const Bno086Sh2TimestampResult result =
      ComputeBno086Sh2Timestamp(Bno086Sh2TimestampInput{1'000'000'000, true, 120, true, 17});

  EXPECT_EQ(result.stamp_ns, 989'700'000);
}

TEST(Bno086Sh2Timestamp, signedTimebaseDeltaCanMoveStampAfterInterrupt)
{
  const Bno086Sh2TimestampResult result =
      ComputeBno086Sh2Timestamp(Bno086Sh2TimestampInput{1'000'000'000, true, -20, true, 5});

  EXPECT_EQ(result.stamp_ns, 1'002'500'000);
}

TEST(Bno086Sh2Timestamp, missingTimebaseUsesInterruptPlusDelayFallback)
{
  const Bno086Sh2TimestampResult result =
      ComputeBno086Sh2Timestamp(Bno086Sh2TimestampInput{1'000'000'000, false, 0, true, 17});

  EXPECT_EQ(result.stamp_ns, 1'001'700'000);
  EXPECT_FALSE(result.used_timebase_reference);
  EXPECT_TRUE(result.used_report_delay);
  EXPECT_TRUE(result.used_missing_timebase_fallback);
}

TEST(Bno086OutputStampGate, duplicateAndNonmonotonicStampsAreDropped)
{
  Bno086OutputStampGate gate;

  EXPECT_TRUE(gate.Check(1'000).should_publish);

  const Bno086OutputStampGateResult duplicate = gate.Check(1'000);
  EXPECT_FALSE(duplicate.should_publish);
  EXPECT_TRUE(duplicate.has_previous_stamp);
  EXPECT_EQ(duplicate.previous_stamp_ns, 1'000);
  EXPECT_EQ(duplicate.current_stamp_ns, 1'000);
  EXPECT_EQ(duplicate.delta_ns, 0);
  EXPECT_TRUE(duplicate.duplicate_stamp);
  EXPECT_FALSE(duplicate.backward_stamp);
  EXPECT_FALSE(duplicate.nonmonotonic_stamp);

  const Bno086OutputStampGateResult older = gate.Check(999);
  EXPECT_FALSE(older.should_publish);
  EXPECT_TRUE(older.has_previous_stamp);
  EXPECT_EQ(older.previous_stamp_ns, 1'000);
  EXPECT_EQ(older.current_stamp_ns, 999);
  EXPECT_EQ(older.delta_ns, -1);
  EXPECT_FALSE(older.duplicate_stamp);
  EXPECT_TRUE(older.backward_stamp);
  EXPECT_TRUE(older.nonmonotonic_stamp);

  EXPECT_TRUE(gate.Check(1'001).should_publish);
}

TEST(Bno086ReportSequenceDiagnostics, gapIsCountedButDoesNotAffectTimestamp)
{
  Bno086ReportSequenceDiagnostics diagnostics;

  const Bno086SequenceUpdate first = diagnostics.Update(ReportId::Accelerometer, 10);
  EXPECT_FALSE(first.duplicate_sequence);
  EXPECT_FALSE(first.sequence_gap);
  EXPECT_EQ(first.sequence_delta, 0);

  const Bno086SequenceUpdate gap = diagnostics.Update(ReportId::Accelerometer, 13);
  EXPECT_FALSE(gap.duplicate_sequence);
  EXPECT_TRUE(gap.sequence_gap);
  EXPECT_EQ(gap.sequence_delta, 3);

  const Bno086Sh2TimestampResult result =
      ComputeBno086Sh2Timestamp(Bno086Sh2TimestampInput{1'000'000'000, true, 120, true, 17});
  EXPECT_EQ(result.stamp_ns, 989'700'000);
}

TEST(Bno086ReportSequenceDiagnostics, duplicateSequenceIsCounted)
{
  Bno086ReportSequenceDiagnostics diagnostics;

  (void)diagnostics.Update(ReportId::RotationVector, 42);
  const Bno086SequenceUpdate duplicate = diagnostics.Update(ReportId::RotationVector, 42);

  EXPECT_TRUE(duplicate.duplicate_sequence);
  EXPECT_FALSE(duplicate.sequence_gap);
  EXPECT_EQ(duplicate.sequence_delta, 0);
}
