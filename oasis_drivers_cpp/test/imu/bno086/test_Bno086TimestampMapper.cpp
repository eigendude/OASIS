/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/Bno086TimestampMapper.hpp"
#include "imu/bno086/Bno086TimestampNormalizer.hpp"

#include <cstdint>
#include <limits>
#include <optional>

#include <gtest/gtest.h>

using namespace OASIS::IMU::BNO086;

namespace
{
TimestampMappingInput Input(std::uint32_t base_timestamp_us,
                            std::uint32_t delay_us,
                            int64_t packet_host_stamp_ns,
                            std::uint8_t sequence = 0)
{
  return TimestampMappingInput{
      true, base_timestamp_us, true, delay_us, sequence, packet_host_stamp_ns,
  };
}
} // namespace

TEST(Bno086TimestampMapper, initialMappingUsesHostDerivedOffset)
{
  Bno086TimestampMapper mapper;

  const TimestampMappingResult result = mapper.Map(Input(1'000'000, 0, 10'000'000'000));

  EXPECT_EQ(result.stamp_ns, 10'000'000'000);
  EXPECT_TRUE(result.used_device_time);
  EXPECT_TRUE(result.initialized_offset);
  EXPECT_TRUE(result.reanchored_offset);
  EXPECT_EQ(result.reanchor_reason, TimestampReanchorReason::Startup);
  EXPECT_FALSE(result.rejected_implausible_mapping);
}

TEST(Bno086TimestampMapper, delaySubtractsFromMappedBase)
{
  Bno086TimestampMapper mapper;

  const TimestampMappingResult result = mapper.Map(Input(1'000'000, 8'000, 10'000'000'000));

  EXPECT_EQ(result.stamp_ns, 9'992'000'000);
  EXPECT_TRUE(result.used_device_time);
}

TEST(Bno086TimestampMapper, stableOffsetIgnoresBurstyHostReceiveTime)
{
  Bno086TimestampMapper mapper;

  ASSERT_EQ(mapper.Map(Input(1'000'000, 0, 10'000'000'000, 1)).stamp_ns, 10'000'000'000);

  const TimestampMappingResult second = mapper.Map(Input(1'008'000, 0, 10'000'000'000, 2));

  EXPECT_EQ(second.stamp_ns, 10'008'000'000);
  EXPECT_FALSE(second.reanchored_offset);
  EXPECT_FALSE(second.detected_wrap_or_reset);
  EXPECT_FALSE(second.rejected_implausible_mapping);
}

TEST(Bno086TimestampMapper, repeatedLocalBackwardBaseDoesNotReanchorAsReset)
{
  Bno086TimestampMapper mapper;

  ASSERT_EQ(mapper.Map(Input(1'000'000, 0, 10'000'000'000, 1)).stamp_ns, 10'000'000'000);

  const TimestampMappingResult second = mapper.Map(Input(995'000, 0, 10'008'000'000, 2));
  const TimestampMappingResult third = mapper.Map(Input(990'000, 0, 10'016'000'000, 3));

  EXPECT_EQ(second.stamp_ns, 9'995'000'000);
  EXPECT_EQ(third.stamp_ns, 9'990'000'000);
  EXPECT_FALSE(second.detected_wrap_or_reset);
  EXPECT_FALSE(third.detected_wrap_or_reset);
  EXPECT_FALSE(second.reanchored_offset);
  EXPECT_FALSE(third.reanchored_offset);
  EXPECT_EQ(second.reanchor_reason, TimestampReanchorReason::None);
  EXPECT_EQ(third.reanchor_reason, TimestampReanchorReason::None);
}

TEST(Bno086TimestampMapper, implausibleDriftFromSmallBackwardBaseDoesNotReanchor)
{
  Bno086TimestampMapper mapper;

  ASSERT_EQ(mapper.Map(Input(1'000'000, 0, 10'000'000'000, 1)).stamp_ns, 10'000'000'000);

  const TimestampMappingResult drift = mapper.Map(Input(998'000, 0, 12'000'000'000, 2));

  EXPECT_EQ(drift.stamp_ns, 9'998'000'000);
  EXPECT_TRUE(drift.rejected_implausible_mapping);
  EXPECT_FALSE(drift.reanchored_offset);
  EXPECT_FALSE(drift.detected_wrap_or_reset);
  EXPECT_EQ(drift.reanchor_reason, TimestampReanchorReason::None);

  const TimestampMappingResult afterDrift = mapper.Map(Input(1'010'000, 0, 12'010'000'000, 3));

  EXPECT_EQ(afterDrift.stamp_ns, 10'010'000'000);
  EXPECT_FALSE(afterDrift.reanchored_offset);
}

TEST(Bno086TimestampMapper, forwardBaseWithHugeHostDriftDoesNotReanchor)
{
  Bno086TimestampMapper mapper;

  ASSERT_EQ(mapper.Map(Input(1'000'000, 0, 10'000'000'000, 1)).stamp_ns, 10'000'000'000);

  const TimestampMappingResult drift = mapper.Map(Input(1'025'400, 0, 12'800'000'000, 2));

  EXPECT_EQ(drift.stamp_ns, 10'025'400'000);
  EXPECT_TRUE(drift.rejected_implausible_mapping);
  EXPECT_FALSE(drift.reanchored_offset);
  EXPECT_FALSE(drift.detected_wrap_or_reset);
  EXPECT_EQ(drift.reanchor_reason, TimestampReanchorReason::None);
}

TEST(Bno086TimestampMapper, smallBackwardBaseNeverTriggersReset)
{
  Bno086TimestampMapper mapper;

  ASSERT_EQ(mapper.Map(Input(1'000'000, 0, 10'000'000'000, 1)).stamp_ns, 10'000'000'000);

  const TimestampMappingResult twoMillisBack = mapper.Map(Input(998'000, 0, 10'600'000'000, 2));
  const TimestampMappingResult hundredMillisBack = mapper.Map(Input(898'000, 0, 11'600'000'000, 3));

  EXPECT_FALSE(twoMillisBack.detected_wrap_or_reset);
  EXPECT_FALSE(hundredMillisBack.detected_wrap_or_reset);
  EXPECT_FALSE(twoMillisBack.reanchored_offset);
  EXPECT_FALSE(hundredMillisBack.reanchored_offset);
  EXPECT_EQ(twoMillisBack.reanchor_reason, TimestampReanchorReason::None);
  EXPECT_EQ(hundredMillisBack.reanchor_reason, TimestampReanchorReason::None);
}

TEST(Bno086TimestampMapper, batchedReportsUseDeviceCadence)
{
  Bno086TimestampMapper mapper;

  const int64_t packetHostStampNs = 10'000'000'000;
  const TimestampMappingResult first = mapper.Map(Input(1'000'000, 16'000, packetHostStampNs, 1));
  const TimestampMappingResult second = mapper.Map(Input(1'000'000, 8'000, packetHostStampNs, 2));
  const TimestampMappingResult third = mapper.Map(Input(1'000'000, 0, packetHostStampNs, 3));

  EXPECT_EQ(first.stamp_ns, 9'984'000'000);
  EXPECT_EQ(second.stamp_ns, 9'992'000'000);
  EXPECT_EQ(third.stamp_ns, 10'000'000'000);
  EXPECT_EQ(second.stamp_ns - first.stamp_ns, 8'000'000);
  EXPECT_EQ(third.stamp_ns - second.stamp_ns, 8'000'000);
}

TEST(Bno086TimestampMapper, hostReceiveOrderDoesNotDriveDeviceSampleTime)
{
  Bno086TimestampMapper mapper;

  ASSERT_EQ(mapper.Map(Input(1'000'000, 0, 10'000'000'000, 1)).stamp_ns, 10'000'000'000);

  const TimestampMappingResult second = mapper.Map(Input(1'020'000, 0, 9'990'000'000, 2));

  EXPECT_EQ(second.stamp_ns, 10'020'000'000);
  EXPECT_FALSE(second.reanchored_offset);
  EXPECT_FALSE(second.rejected_implausible_mapping);
}

TEST(Bno086TimestampMapper, largeBackwardJumpReanchorsAsDeviceReset)
{
  Bno086TimestampMapper mapper;

  ASSERT_EQ(mapper.Map(Input(2'000'000, 0, 10'000'000'000, 1)).stamp_ns, 10'000'000'000);

  const TimestampMappingResult reset = mapper.Map(Input(100'000, 0, 12'000'000'000, 2));

  EXPECT_EQ(reset.stamp_ns, 12'000'000'000);
  EXPECT_TRUE(reset.detected_wrap_or_reset);
  EXPECT_TRUE(reset.reanchored_offset);
  EXPECT_EQ(reset.reanchor_reason, TimestampReanchorReason::Reset);
}

TEST(Bno086TimestampMapper, smallBackwardModuloStepExtendsWrap)
{
  Bno086TimestampMapper mapper;

  constexpr std::uint32_t kNearWrapUs = std::numeric_limits<std::uint32_t>::max() - 1'999U;
  ASSERT_EQ(mapper.Map(Input(kNearWrapUs, 0, 10'000'000'000, 1)).stamp_ns, 10'000'000'000);

  const TimestampMappingResult wrapped = mapper.Map(Input(1'000, 0, 10'000'000'000, 2));

  EXPECT_EQ(wrapped.stamp_ns, 10'003'000'000);
  EXPECT_TRUE(wrapped.detected_wrap_or_reset);
  EXPECT_FALSE(wrapped.reanchored_offset);
  EXPECT_EQ(wrapped.reanchor_reason, TimestampReanchorReason::Wrap);
}

TEST(Bno086TimestampMapper, missingBaseTimestampFallsBackToHostMinusDelay)
{
  Bno086TimestampMapper mapper;

  TimestampMappingInput input = Input(0, 8'000, 10'000'000'000);
  input.has_base_timestamp = false;

  const TimestampMappingResult result = mapper.Map(input);

  EXPECT_EQ(result.stamp_ns, 9'992'000'000);
  EXPECT_FALSE(result.used_device_time);
}

TEST(Bno086TimestampMapper, mappedStampsDoNotTriggerNormalizerRepair)
{
  Bno086TimestampMapper mapper;
  Bno086TimestampNormalizer normalizer;

  constexpr int64_t kMappedTimestampFutureSlopNs = 50'000'000;
  const TimestampMappingResult firstMapped = mapper.Map(Input(1'000'000, 0, 10'000'000'000, 1));
  const TimestampNormalizationResult firstNormalized =
      normalizer.Normalize(TimestampSample{1, firstMapped.stamp_ns, 10'000'000'000,
                                           std::make_optional(kMappedTimestampFutureSlopNs)},
                           8'000'000);

  const TimestampMappingResult secondMapped = mapper.Map(Input(1'008'000, 0, 10'000'000'000, 2));
  const TimestampNormalizationResult secondNormalized =
      normalizer.Normalize(TimestampSample{2, secondMapped.stamp_ns, 10'000'000'000,
                                           std::make_optional(kMappedTimestampFutureSlopNs)},
                           8'000'000);

  EXPECT_EQ(firstNormalized.stamp_ns, 10'000'000'000);
  EXPECT_EQ(secondNormalized.stamp_ns, 10'008'000'000);
  EXPECT_FALSE(secondNormalized.reconstruction_reset);
  EXPECT_FALSE(secondNormalized.repaired_nonmonotonic);
  EXPECT_FALSE(secondNormalized.interval_repair_bounded_to_legacy);
  EXPECT_NE(secondNormalized.stamp_ns, firstNormalized.stamp_ns + 1);
}
