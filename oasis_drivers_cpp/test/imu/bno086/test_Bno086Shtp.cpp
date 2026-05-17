/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/sh2/Bno086Shtp.hpp"

#include <cstdint>
#include <deque>
#include <vector>

#include <gtest/gtest.h>

using namespace OASIS::IMU::BNO086;

namespace
{
class RecordingTransport : public Bno086Transport
{
public:
  bool WritePacket(std::uint8_t channel, const std::vector<std::uint8_t>& payload) override
  {
    channels.emplace_back(channel);
    payloads.emplace_back(payload);
    return true;
  }

  bool ReadPacket(Bno086ShtpPacket& packet, int timeout_ms) override
  {
    read_timeouts.emplace_back(timeout_ms);

    if (packets.empty())
      return false;

    packet = packets.front();
    packets.pop_front();
    return true;
  }

  std::vector<std::uint8_t> channels;
  std::vector<std::vector<std::uint8_t>> payloads;
  std::vector<int> read_timeouts;
  std::deque<Bno086ShtpPacket> packets;
};

std::vector<std::uint8_t> AccelerometerReport(std::uint8_t sequence)
{
  return {
      static_cast<std::uint8_t>(ReportId::Accelerometer),
      sequence,
      0x00,
      0x00,
      0x00,
      0x00,
      0x00,
      0x00,
      0x00,
      0x00,
  };
}

std::uint32_t ReadIntervalUs(const std::vector<std::uint8_t>& payload)
{
  return static_cast<std::uint32_t>(payload[5]) | (static_cast<std::uint32_t>(payload[6]) << 8) |
         (static_cast<std::uint32_t>(payload[7]) << 16) |
         (static_cast<std::uint32_t>(payload[8]) << 24);
}

std::uint32_t ReadBatchIntervalUs(const std::vector<std::uint8_t>& payload)
{
  return static_cast<std::uint32_t>(payload[9]) | (static_cast<std::uint32_t>(payload[10]) << 8) |
         (static_cast<std::uint32_t>(payload[11]) << 16) |
         (static_cast<std::uint32_t>(payload[12]) << 24);
}

void WriteU32(std::vector<std::uint8_t>& payload, std::size_t offset, std::uint32_t value)
{
  payload[offset] = static_cast<std::uint8_t>(value & 0xFF);
  payload[offset + 1] = static_cast<std::uint8_t>((value >> 8) & 0xFF);
  payload[offset + 2] = static_cast<std::uint8_t>((value >> 16) & 0xFF);
  payload[offset + 3] = static_cast<std::uint8_t>((value >> 24) & 0xFF);
}

std::vector<std::uint8_t> FeatureResponsePayload(ReportId report_id,
                                                 std::uint32_t actual_interval_us,
                                                 std::uint32_t actual_batch_us)
{
  std::vector<std::uint8_t> payload(17, 0);
  payload[0] = kShtpGetFeatureResponse;
  payload[1] = static_cast<std::uint8_t>(report_id);
  payload[2] = 0xA5;
  payload[3] = 0x34;
  payload[4] = 0x12;
  WriteU32(payload, 5, actual_interval_us);
  WriteU32(payload, 9, actual_batch_us);
  WriteU32(payload, 13, 0x12345678);
  return payload;
}

std::vector<std::uint8_t> GravityReport(std::uint8_t sequence)
{
  return {
      static_cast<std::uint8_t>(ReportId::Gravity),
      sequence,
      0x00,
      0x00,
      0x00,
      0x00,
      0x00,
      0x00,
      0x00,
      0x00,
  };
}

std::vector<std::uint8_t> LinearAccelerationReport(std::uint8_t sequence)
{
  return {
      static_cast<std::uint8_t>(ReportId::LinearAcceleration),
      sequence,
      0x00,
      0x00,
      0x00,
      0x00,
      0x00,
      0x00,
      0x00,
      0x00,
  };
}

TEST(Bno086Shtp, pollDistinguishesPhysicalPacketReadFromPendingEvent)
{
  RecordingTransport transport;
  Bno086ShtpPacket packet;
  packet.channel = 3;
  packet.payload = AccelerometerReport(1);
  const std::vector<std::uint8_t> secondReport = AccelerometerReport(2);
  packet.payload.insert(packet.payload.end(), secondReport.begin(), secondReport.end());
  transport.packets.emplace_back(packet);

  Bno086Shtp shtp{transport};

  const Bno086Shtp::PollResult firstResult = shtp.Poll(5);

  ASSERT_EQ(firstResult.status, Bno086Shtp::PollStatus::SensorEvent);
  ASSERT_TRUE(firstResult.event.has_value());
  EXPECT_TRUE(firstResult.read_physical_packet);
  EXPECT_FALSE(firstResult.dequeued_pending_event);
  EXPECT_EQ(firstResult.event->sequence, 1);
  EXPECT_EQ(transport.read_timeouts.size(), 1U);

  const Bno086Shtp::PollResult secondResult = shtp.Poll(5);

  ASSERT_EQ(secondResult.status, Bno086Shtp::PollStatus::SensorEvent);
  ASSERT_TRUE(secondResult.event.has_value());
  EXPECT_FALSE(secondResult.read_physical_packet);
  EXPECT_TRUE(secondResult.dequeued_pending_event);
  EXPECT_EQ(secondResult.event->sequence, 2);
  EXPECT_EQ(transport.read_timeouts.size(), 1U);
}

TEST(Bno086Shtp, pollMarksPhysicalControlPacketRead)
{
  RecordingTransport transport;
  Bno086ShtpPacket packet;
  packet.channel = 2;
  packet.payload = {0x00};
  transport.packets.emplace_back(packet);

  Bno086Shtp shtp{transport};

  const Bno086Shtp::PollResult result = shtp.Poll(5);

  EXPECT_EQ(result.status, Bno086Shtp::PollStatus::PacketHandled);
  EXPECT_TRUE(result.read_physical_packet);
  EXPECT_FALSE(result.dequeued_pending_event);
  EXPECT_TRUE(result.handled_control_packet);
  EXPECT_FALSE(result.event.has_value());
}
} // namespace

TEST(Bno086Shtp, configuresStaticPerReportIntervalsAndDefaultBatchIntervals)
{
  RecordingTransport transport;
  Bno086Shtp shtp{transport};

  Bno086ShtpConfig config;
  config.report_rate_hz = 10.0;
  config.rotation_vector_rate_hz = 100.0;
  config.gyro_rate_hz = 100.0;
  config.accelerometer_rate_hz = 100.0;
  config.linear_acceleration_rate_hz = 50.0;
  config.gravity_rate_hz = 25.0;

  ASSERT_TRUE(shtp.Configure(config));

  const std::vector<FeatureConfiguration>& features = shtp.GetFeatureConfigurations();
  ASSERT_EQ(features.size(), 5U);
  EXPECT_EQ(features[0].report_id, ReportId::RotationVector);
  EXPECT_EQ(features[0].requested_interval_us, 10'000U);
  EXPECT_EQ(features[0].requested_batch_interval_us, 20'000U);
  EXPECT_TRUE(features[0].enabled);
  EXPECT_EQ(features[1].report_id, ReportId::GyroscopeCalibrated);
  EXPECT_EQ(features[1].requested_interval_us, 10'000U);
  EXPECT_EQ(features[1].requested_batch_interval_us, 20'000U);
  EXPECT_TRUE(features[1].enabled);
  EXPECT_EQ(features[2].report_id, ReportId::LinearAcceleration);
  EXPECT_EQ(features[2].requested_interval_us, 20'000U);
  EXPECT_EQ(features[2].requested_batch_interval_us, 20'000U);
  EXPECT_TRUE(features[2].enabled);
  EXPECT_EQ(features[3].report_id, ReportId::Accelerometer);
  EXPECT_EQ(features[3].requested_interval_us, 10'000U);
  EXPECT_EQ(features[3].requested_batch_interval_us, 20'000U);
  EXPECT_TRUE(features[3].enabled);
  EXPECT_EQ(features[4].report_id, ReportId::Gravity);
  EXPECT_EQ(features[4].requested_interval_us, 40'000U);
  EXPECT_EQ(features[4].requested_batch_interval_us, 40'000U);
  EXPECT_TRUE(features[4].enabled);

  ASSERT_EQ(transport.payloads.size(), 10U);
  EXPECT_EQ(ReadIntervalUs(transport.payloads[0]), 10'000U);
  EXPECT_EQ(ReadBatchIntervalUs(transport.payloads[0]), 20'000U);
  EXPECT_EQ(transport.payloads[1], (std::vector<std::uint8_t>{
                                       kShtpGetFeatureRequest,
                                       static_cast<std::uint8_t>(ReportId::RotationVector),
                                   }));
  EXPECT_EQ(ReadIntervalUs(transport.payloads[2]), 10'000U);
  EXPECT_EQ(ReadBatchIntervalUs(transport.payloads[2]), 20'000U);
  EXPECT_EQ(ReadIntervalUs(transport.payloads[4]), 20'000U);
  EXPECT_EQ(ReadBatchIntervalUs(transport.payloads[4]), 20'000U);
  EXPECT_EQ(ReadIntervalUs(transport.payloads[6]), 10'000U);
  EXPECT_EQ(ReadBatchIntervalUs(transport.payloads[6]), 20'000U);
  EXPECT_EQ(ReadIntervalUs(transport.payloads[8]), 40'000U);
  EXPECT_EQ(ReadBatchIntervalUs(transport.payloads[8]), 40'000U);
}

TEST(Bno086Shtp, explicitZeroBatchIntervalDisablesBatchingForReport)
{
  RecordingTransport transport;
  Bno086Shtp shtp{transport};

  Bno086ShtpConfig config;
  config.gyro_batch_interval_us = 0;

  ASSERT_TRUE(shtp.Configure(config));

  const std::vector<FeatureConfiguration>& features = shtp.GetFeatureConfigurations();
  ASSERT_EQ(features.size(), 5U);
  EXPECT_EQ(features[1].report_id, ReportId::GyroscopeCalibrated);
  EXPECT_EQ(features[1].requested_interval_us, 10'000U);
  EXPECT_EQ(features[1].requested_batch_interval_us, 0U);
  EXPECT_TRUE(features[1].enabled);

  ASSERT_EQ(transport.payloads.size(), 10U);
  EXPECT_EQ(ReadIntervalUs(transport.payloads[2]), 10'000U);
  EXPECT_EQ(ReadBatchIntervalUs(transport.payloads[2]), 0U);
}

TEST(Bno086Shtp, disablesOptionalReportsWithZeroIntervalAndBatch)
{
  RecordingTransport transport;
  Bno086Shtp shtp{transport};

  Bno086ShtpConfig config;
  config.enable_linear_acceleration_report = false;
  config.enable_gravity_report = false;

  ASSERT_TRUE(shtp.Configure(config));

  const std::vector<FeatureConfiguration>& features = shtp.GetFeatureConfigurations();
  ASSERT_EQ(features.size(), 5U);
  EXPECT_EQ(features[2].report_id, ReportId::LinearAcceleration);
  EXPECT_EQ(features[2].requested_interval_us, 0U);
  EXPECT_EQ(features[2].requested_batch_interval_us, 0U);
  EXPECT_FALSE(features[2].enabled);
  EXPECT_EQ(features[4].report_id, ReportId::Gravity);
  EXPECT_EQ(features[4].requested_interval_us, 0U);
  EXPECT_EQ(features[4].requested_batch_interval_us, 0U);
  EXPECT_FALSE(features[4].enabled);

  ASSERT_EQ(transport.payloads.size(), 10U);
  EXPECT_EQ(ReadIntervalUs(transport.payloads[4]), 0U);
  EXPECT_EQ(ReadBatchIntervalUs(transport.payloads[4]), 0U);
  EXPECT_EQ(ReadIntervalUs(transport.payloads[8]), 0U);
  EXPECT_EQ(ReadBatchIntervalUs(transport.payloads[8]), 0U);
}

TEST(Bno086Shtp, parsesFeatureResponseIntervalsAndBatchingState)
{
  RecordingTransport transport;
  Bno086Shtp shtp{transport};

  Bno086ShtpConfig config;
  ASSERT_TRUE(shtp.Configure(config));

  Bno086ShtpPacket packet;
  packet.channel = 2;
  packet.payload = FeatureResponsePayload(ReportId::Accelerometer, 10'000, 20'000);
  transport.packets.emplace_back(packet);

  const Bno086Shtp::PollResult result = shtp.Poll(5);
  EXPECT_EQ(result.status, Bno086Shtp::PollStatus::PacketHandled);

  const std::vector<FeatureResponse> responses = shtp.TakeFeatureResponses();
  ASSERT_EQ(responses.size(), 1U);
  EXPECT_EQ(responses[0].report_id, ReportId::Accelerometer);
  EXPECT_EQ(responses[0].feature_flags, 0xA5);
  EXPECT_EQ(responses[0].change_sensitivity, 0x1234);
  EXPECT_EQ(responses[0].report_interval_us, 10'000U);
  EXPECT_EQ(responses[0].batch_interval_us, 20'000U);
  EXPECT_EQ(responses[0].sensor_specific_config, 0x12345678U);
  EXPECT_TRUE(IsFeatureBatchingActive(responses[0]));
}

TEST(Bno086Shtp, zeroActualBatchIsNotBatchingActive)
{
  FeatureResponse response;
  response.batch_interval_us = 0;
  EXPECT_FALSE(IsFeatureBatchingActive(response));

  response.batch_interval_us = 1;
  EXPECT_TRUE(IsFeatureBatchingActive(response));
}

TEST(Bno086Shtp, controlParserScansFeatureResponseAtOffsetZero)
{
  RecordingTransport transport;
  Bno086Shtp shtp{transport};

  Bno086ShtpConfig config;
  ASSERT_TRUE(shtp.Configure(config));

  Bno086ShtpPacket packet;
  packet.channel = 2;
  packet.payload = FeatureResponsePayload(ReportId::GyroscopeCalibrated, 11'000, 22'000);
  transport.packets.emplace_back(packet);

  EXPECT_EQ(shtp.Poll(5).status, Bno086Shtp::PollStatus::PacketHandled);
  const std::vector<FeatureResponse> responses = shtp.TakeFeatureResponses();
  ASSERT_EQ(responses.size(), 1U);
  EXPECT_EQ(responses[0].report_id, ReportId::GyroscopeCalibrated);
}

TEST(Bno086Shtp, controlParserScansFeatureResponseAfterUnknownByte)
{
  RecordingTransport transport;
  Bno086Shtp shtp{transport};

  Bno086ShtpConfig config;
  ASSERT_TRUE(shtp.Configure(config));

  Bno086ShtpPacket packet;
  packet.channel = 2;
  packet.payload = {0xAA};
  const std::vector<std::uint8_t> response =
      FeatureResponsePayload(ReportId::RotationVector, 10'000, 20'000);
  packet.payload.insert(packet.payload.end(), response.begin(), response.end());
  transport.packets.emplace_back(packet);

  EXPECT_EQ(shtp.Poll(5).status, Bno086Shtp::PollStatus::PacketHandled);
  const std::vector<FeatureResponse> responses = shtp.TakeFeatureResponses();
  ASSERT_EQ(responses.size(), 1U);
  EXPECT_EQ(responses[0].report_id, ReportId::RotationVector);
}

TEST(Bno086Shtp, controlParserSkipsProductResponseBeforeFeatureResponse)
{
  RecordingTransport transport;
  Bno086Shtp shtp{transport};

  Bno086ShtpConfig config;
  ASSERT_TRUE(shtp.Configure(config));

  Bno086ShtpPacket packet;
  packet.channel = 2;
  packet.payload = std::vector<std::uint8_t>(16, 0x00);
  packet.payload[0] = 0xF8;
  const std::vector<std::uint8_t> response = FeatureResponsePayload(ReportId::Gravity, 40'000, 0);
  packet.payload.insert(packet.payload.end(), response.begin(), response.end());
  transport.packets.emplace_back(packet);

  EXPECT_EQ(shtp.Poll(5).status, Bno086Shtp::PollStatus::PacketHandled);
  const std::vector<FeatureResponse> responses = shtp.TakeFeatureResponses();
  ASSERT_EQ(responses.size(), 1U);
  EXPECT_EQ(responses[0].report_id, ReportId::Gravity);
}

TEST(Bno086Shtp, shortFeatureResponseDoesNotCrash)
{
  RecordingTransport transport;
  Bno086Shtp shtp{transport};

  Bno086ShtpConfig config;
  ASSERT_TRUE(shtp.Configure(config));

  Bno086ShtpPacket packet;
  packet.channel = 2;
  packet.payload = {kShtpGetFeatureResponse, static_cast<std::uint8_t>(ReportId::Accelerometer)};
  transport.packets.emplace_back(packet);

  EXPECT_EQ(shtp.Poll(5).status, Bno086Shtp::PollStatus::PacketHandled);
  EXPECT_TRUE(shtp.TakeFeatureResponses().empty());
}

TEST(Bno086Shtp, disabledOptionalReportsAreIgnoredIfStalePayloadsArrive)
{
  RecordingTransport transport;
  Bno086Shtp shtp{transport};

  Bno086ShtpConfig config;
  config.enable_linear_acceleration_report = false;
  config.enable_gravity_report = false;
  ASSERT_TRUE(shtp.Configure(config));

  Bno086ShtpPacket linearPacket;
  linearPacket.channel = 3;
  linearPacket.payload = LinearAccelerationReport(1);
  transport.packets.emplace_back(linearPacket);

  Bno086ShtpPacket gravityPacket;
  gravityPacket.channel = 3;
  gravityPacket.payload = GravityReport(1);
  transport.packets.emplace_back(gravityPacket);

  const Bno086Shtp::PollResult linearResult = shtp.Poll(5);
  EXPECT_EQ(linearResult.status, Bno086Shtp::PollStatus::PacketHandled);
  EXPECT_FALSE(linearResult.event.has_value());

  const Bno086Shtp::PollResult gravityResult = shtp.Poll(5);
  EXPECT_EQ(gravityResult.status, Bno086Shtp::PollStatus::PacketHandled);
  EXPECT_FALSE(gravityResult.event.has_value());
}
