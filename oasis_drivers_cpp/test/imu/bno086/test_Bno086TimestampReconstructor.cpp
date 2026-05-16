/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/Bno086TimestampReconstructor.hpp"

#include <cstdint>

#include <gtest/gtest.h>

namespace OASIS::IMU::BNO086
{
namespace
{
SensorEvent EventWithBase(std::uint32_t base_timestamp_us, std::uint16_t delay_us = 0)
{
  SensorEvent event;
  event.has_base_timestamp = true;
  event.base_timestamp_us = base_timestamp_us;
  event.has_delay = true;
  event.delay_us = delay_us;
  return event;
}
} // namespace

TEST(Bno086TimestampReconstructor, perReportDelaySubtractsFromReconstructedStamp)
{
  Bno086TimestampReconstructor reconstructor;

  const std::int64_t stampNs = reconstructor.Reconstruct(EventWithBase(1'000, 300), 1'000'000'000);

  EXPECT_EQ(stampNs, 999'300'000);
  EXPECT_EQ(reconstructor.GetDiagnostics().delay_applied, 1U);
}

TEST(Bno086TimestampReconstructor, timebaseDeltaSubtractsFromPacketHostStamp)
{
  Bno086TimestampReconstructor reconstructor;

  EXPECT_EQ(reconstructor.Reconstruct(EventWithBase(1'000), 1'000'000'000), 999'000'000);
  EXPECT_EQ(reconstructor.Reconstruct(EventWithBase(1'100), 1'000'100'000), 999'000'000);
  EXPECT_EQ(reconstructor.GetDiagnostics().base_resets, 0U);
  EXPECT_EQ(reconstructor.GetDiagnostics().latest_base_delta_us, 1'100);
  EXPECT_EQ(reconstructor.GetDiagnostics().latest_host_delta_us, 0);
  EXPECT_EQ(reconstructor.GetDiagnostics().latest_base_host_error_us, 0);
}

TEST(Bno086TimestampReconstructor, largeTimebaseDeltaResetsToPacketHostStamp)
{
  Bno086TimestampReconstructor reconstructor;

  EXPECT_EQ(reconstructor.Reconstruct(EventWithBase(1'000'001), 1'000'000'000), 1'000'000'000);
  EXPECT_EQ(reconstructor.GetDiagnostics().base_resets, 1U);
  EXPECT_EQ(reconstructor.GetDiagnostics().base_resets_large_delta, 1U);
}

TEST(Bno086TimestampReconstructor, datasheetTimebaseAndDelayExample)
{
  Bno086TimestampReconstructor reconstructor;

  EXPECT_EQ(reconstructor.Reconstruct(EventWithBase(12'000, 1'700), 1'000'000'000), 989'700'000);
}

TEST(Bno086TimestampReconstructor, missingBaseUsesPacketHostStamp)
{
  Bno086TimestampReconstructor reconstructor;
  SensorEvent event;

  const std::int64_t stampNs = reconstructor.Reconstruct(event, 1'234'000'000);

  EXPECT_EQ(stampNs, 1'234'000'000);
  EXPECT_EQ(reconstructor.GetDiagnostics().missing_base, 1U);
}
} // namespace OASIS::IMU::BNO086
