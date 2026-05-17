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

TEST(Bno086Shtp, configuresStaticPerReportIntervals)
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
  EXPECT_EQ(features[1].report_id, ReportId::GyroscopeCalibrated);
  EXPECT_EQ(features[1].requested_interval_us, 10'000U);
  EXPECT_EQ(features[2].report_id, ReportId::LinearAcceleration);
  EXPECT_EQ(features[2].requested_interval_us, 20'000U);
  EXPECT_EQ(features[3].report_id, ReportId::Accelerometer);
  EXPECT_EQ(features[3].requested_interval_us, 10'000U);
  EXPECT_EQ(features[4].report_id, ReportId::Gravity);
  EXPECT_EQ(features[4].requested_interval_us, 40'000U);

  ASSERT_GE(transport.payloads.size(), 9U);
  EXPECT_EQ(ReadIntervalUs(transport.payloads[0]), 10'000U);
  EXPECT_EQ(ReadIntervalUs(transport.payloads[2]), 10'000U);
  EXPECT_EQ(ReadIntervalUs(transport.payloads[4]), 20'000U);
  EXPECT_EQ(ReadIntervalUs(transport.payloads[6]), 10'000U);
  EXPECT_EQ(ReadIntervalUs(transport.payloads[8]), 40'000U);
}

TEST(Bno086Shtp, omitsDisabledGravityReport)
{
  RecordingTransport transport;
  Bno086Shtp shtp{transport};

  Bno086ShtpConfig config;
  config.enable_gravity_report = false;

  ASSERT_TRUE(shtp.Configure(config));

  const std::vector<FeatureConfiguration>& features = shtp.GetFeatureConfigurations();
  ASSERT_EQ(features.size(), 4U);
  for (const FeatureConfiguration& feature : features)
    EXPECT_NE(feature.report_id, ReportId::Gravity);
}
