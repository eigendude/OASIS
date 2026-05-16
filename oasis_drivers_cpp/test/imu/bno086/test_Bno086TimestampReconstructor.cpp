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

  EXPECT_EQ(stampNs, 999'700'000);
  EXPECT_EQ(reconstructor.GetDiagnostics().delay_applied, 1U);
}

TEST(Bno086TimestampReconstructor, forwardBaseDeltaAccepted)
{
  Bno086TimestampReconstructor reconstructor;

  EXPECT_EQ(reconstructor.Reconstruct(EventWithBase(1'000), 1'000'000'000), 1'000'000'000);
  EXPECT_EQ(reconstructor.Reconstruct(EventWithBase(1'100), 1'000'100'000), 1'000'100'000);
  EXPECT_EQ(reconstructor.GetDiagnostics().base_resets, 0U);
  EXPECT_EQ(reconstructor.GetDiagnostics().latest_base_delta_us, 100);
  EXPECT_EQ(reconstructor.GetDiagnostics().latest_host_delta_us, 100);
  EXPECT_EQ(reconstructor.GetDiagnostics().latest_base_host_error_us, 0);
}

TEST(Bno086TimestampReconstructor, backwardBaseTimestampWithoutWrapResets)
{
  Bno086TimestampReconstructor reconstructor;

  EXPECT_EQ(reconstructor.Reconstruct(EventWithBase(2'000), 1'000'000'000), 1'000'000'000);
  EXPECT_EQ(reconstructor.Reconstruct(EventWithBase(1'000), 1'001'000'000), 1'001'000'000);
  EXPECT_EQ(reconstructor.GetDiagnostics().base_resets, 1U);
  EXPECT_EQ(reconstructor.GetDiagnostics().base_resets_negative_delta, 1U);
}

TEST(Bno086TimestampReconstructor, hugeForwardBaseDeltaResets)
{
  Bno086TimestampReconstructor reconstructor;

  EXPECT_EQ(reconstructor.Reconstruct(EventWithBase(1'000), 1'000'000'000), 1'000'000'000);
  EXPECT_EQ(reconstructor.Reconstruct(EventWithBase(2'000'100), 2'000'000'000), 2'000'000'000);
  EXPECT_EQ(reconstructor.GetDiagnostics().base_resets, 1U);
  EXPECT_EQ(reconstructor.GetDiagnostics().base_resets_large_delta, 1U);
}

TEST(Bno086TimestampReconstructor, plausibleUint32WrapAccepted)
{
  Bno086TimestampReconstructor reconstructor;

  constexpr std::uint32_t lastBaseUs = 0xFFFFFF00U;
  constexpr std::uint32_t currentBaseUs = 100U;
  constexpr std::int64_t wrappedDeltaUs = 356;

  EXPECT_EQ(reconstructor.Reconstruct(EventWithBase(lastBaseUs), 1'000'000'000), 1'000'000'000);
  EXPECT_EQ(reconstructor.Reconstruct(EventWithBase(currentBaseUs),
                                      1'000'000'000 + wrappedDeltaUs * 1000),
            1'000'000'000 + wrappedDeltaUs * 1000);
  EXPECT_EQ(reconstructor.GetDiagnostics().base_resets, 0U);
  EXPECT_EQ(reconstructor.GetDiagnostics().base_wraps_accepted, 1U);
  EXPECT_EQ(reconstructor.GetDiagnostics().latest_base_delta_us, wrappedDeltaUs);
}

TEST(Bno086TimestampReconstructor, implausibleWrapResets)
{
  Bno086TimestampReconstructor reconstructor;

  EXPECT_EQ(reconstructor.Reconstruct(EventWithBase(0xFFFFFF00U), 1'000'000'000), 1'000'000'000);
  EXPECT_EQ(reconstructor.Reconstruct(EventWithBase(0x00020000U), 1'001'000'000), 1'001'000'000);
  EXPECT_EQ(reconstructor.GetDiagnostics().base_resets, 1U);
  EXPECT_EQ(reconstructor.GetDiagnostics().base_resets_negative_delta, 1U);
}

TEST(Bno086TimestampReconstructor, baseVsHostMismatchResets)
{
  Bno086TimestampReconstructor reconstructor;

  EXPECT_EQ(reconstructor.Reconstruct(EventWithBase(1'000), 1'000'000'000), 1'000'000'000);
  EXPECT_EQ(reconstructor.Reconstruct(EventWithBase(1'200), 2'000'000'000), 2'000'000'000);
  EXPECT_EQ(reconstructor.GetDiagnostics().base_resets, 1U);
  EXPECT_EQ(reconstructor.GetDiagnostics().base_resets_host_mismatch, 1U);
  EXPECT_EQ(reconstructor.GetDiagnostics().latest_base_delta_us, 200);
  EXPECT_EQ(reconstructor.GetDiagnostics().latest_host_delta_us, 1'000'000);
  EXPECT_EQ(reconstructor.GetDiagnostics().latest_base_host_error_us, -999'800);
}

TEST(Bno086TimestampReconstructor, baseVsHostSmallMismatchAccepted)
{
  Bno086TimestampReconstructor reconstructor;

  EXPECT_EQ(reconstructor.Reconstruct(EventWithBase(1'000), 1'000'000'000), 1'000'000'000);
  EXPECT_EQ(reconstructor.Reconstruct(EventWithBase(1'100), 1'000'150'000), 1'000'100'000);
  EXPECT_EQ(reconstructor.GetDiagnostics().base_resets, 0U);
  EXPECT_EQ(reconstructor.GetDiagnostics().latest_base_delta_us, 100);
  EXPECT_EQ(reconstructor.GetDiagnostics().latest_host_delta_us, 150);
  EXPECT_EQ(reconstructor.GetDiagnostics().latest_base_host_error_us, -50);
}

TEST(Bno086TimestampReconstructor, resetAnchorsCurrentBaseToPacketHostStamp)
{
  Bno086TimestampReconstructor reconstructor;

  EXPECT_EQ(reconstructor.Reconstruct(EventWithBase(1'000), 1'000'000'000), 1'000'000'000);
  EXPECT_EQ(reconstructor.Reconstruct(EventWithBase(1'200), 2'000'000'000), 2'000'000'000);
  EXPECT_EQ(reconstructor.Reconstruct(EventWithBase(1'300), 2'000'100'000), 2'000'100'000);
  EXPECT_EQ(reconstructor.GetDiagnostics().base_resets, 1U);
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
