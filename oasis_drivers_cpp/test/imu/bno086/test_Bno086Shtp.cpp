/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/Bno086Shtp.hpp"

#include <cstdint>
#include <deque>
#include <vector>

#include <gtest/gtest.h>

namespace OASIS::IMU::BNO086
{
namespace
{
constexpr std::uint8_t kReportChannel = 3;

class FakeTransport : public Bno086Transport
{
public:
  void PushPacket(const std::vector<std::uint8_t>& payload)
  {
    Bno086ShtpPacket packet;
    packet.channel = kReportChannel;
    packet.payload = payload;
    m_packets.emplace_back(packet);
  }

  bool ReadPacket(Bno086ShtpPacket& packet, int timeout_ms) override
  {
    (void)timeout_ms;

    if (m_packets.empty())
      return false;

    packet = m_packets.front();
    m_packets.pop_front();
    return true;
  }

  bool WritePacket(std::uint8_t channel, const std::vector<std::uint8_t>& payload) override
  {
    (void)channel;
    (void)payload;
    return true;
  }

private:
  std::deque<Bno086ShtpPacket> m_packets;
};

void AppendU32(std::vector<std::uint8_t>& payload, std::uint32_t value)
{
  payload.emplace_back(static_cast<std::uint8_t>(value & 0xFFU));
  payload.emplace_back(static_cast<std::uint8_t>((value >> 8) & 0xFFU));
  payload.emplace_back(static_cast<std::uint8_t>((value >> 16) & 0xFFU));
  payload.emplace_back(static_cast<std::uint8_t>((value >> 24) & 0xFFU));
}

void AppendBaseTimestamp(std::vector<std::uint8_t>& payload, std::uint32_t base_ticks)
{
  payload.emplace_back(kShtpReportBaseTimestamp);
  AppendU32(payload, base_ticks);
}

void AppendTimestampRebase(std::vector<std::uint8_t>& payload, std::uint32_t delta_ticks)
{
  payload.emplace_back(kShtpReportTimestampRebase);
  AppendU32(payload, delta_ticks);
}

void AppendSensorReport(std::vector<std::uint8_t>& payload,
                        ReportId report_id,
                        std::uint8_t sequence,
                        std::uint16_t delay_ticks = 0)
{
  payload.emplace_back(static_cast<std::uint8_t>(report_id));
  payload.emplace_back(sequence);
  payload.emplace_back(static_cast<std::uint8_t>((delay_ticks >> 8) << 2));
  payload.emplace_back(static_cast<std::uint8_t>(delay_ticks & 0xFFU));

  const std::size_t valueBytes = report_id == ReportId::RotationVector ? 10 : 6;
  for (std::size_t i = 0; i < valueBytes; ++i)
    payload.emplace_back(static_cast<std::uint8_t>(i));
}
} // namespace

TEST(Bno086Shtp, pollReturnsMultiReportPayloadAsTimestampedBatch)
{
  FakeTransport transport;
  Bno086Shtp shtp(transport);

  std::vector<std::uint8_t> payload;
  AppendBaseTimestamp(payload, 10);
  AppendSensorReport(payload, ReportId::Accelerometer, 1);
  AppendSensorReport(payload, ReportId::GyroscopeCalibrated, 2);
  transport.PushPacket(payload);

  std::vector<TimestampedSensorEvent> events;
  const Bno086Shtp::PollStatus status = shtp.Poll(events, 0, 1'000'000'000);

  ASSERT_EQ(status, Bno086Shtp::PollStatus::SensorEvent);
  ASSERT_EQ(events.size(), 2U);
  EXPECT_EQ(events[0].event.report_id, ReportId::Accelerometer);
  EXPECT_EQ(events[1].event.report_id, ReportId::GyroscopeCalibrated);
  EXPECT_EQ(events[0].stamp_ns, 1'000'000'000);
  EXPECT_EQ(events[1].stamp_ns, 1'000'000'000);
}

TEST(Bno086Shtp, additionalReportsAreNotReturnedFromPendingQueueLater)
{
  FakeTransport transport;
  Bno086Shtp shtp(transport);

  std::vector<std::uint8_t> payload;
  AppendBaseTimestamp(payload, 10);
  AppendSensorReport(payload, ReportId::Accelerometer, 1);
  AppendSensorReport(payload, ReportId::GyroscopeCalibrated, 2);
  transport.PushPacket(payload);

  std::vector<TimestampedSensorEvent> events;
  EXPECT_EQ(shtp.Poll(events, 0, 1'000'000'000), Bno086Shtp::PollStatus::SensorEvent);
  ASSERT_EQ(events.size(), 2U);

  EXPECT_EQ(shtp.Poll(events, 0, 2'000'000'000), Bno086Shtp::PollStatus::Timeout);
  EXPECT_TRUE(events.empty());
}

TEST(Bno086Shtp, perReportDelaySubtractsBeforeEventsLeaveShtp)
{
  FakeTransport transport;
  Bno086Shtp shtp(transport);

  std::vector<std::uint8_t> payload;
  AppendBaseTimestamp(payload, 10);
  AppendSensorReport(payload, ReportId::Accelerometer, 1, 3);
  transport.PushPacket(payload);

  std::vector<TimestampedSensorEvent> events;
  ASSERT_EQ(shtp.Poll(events, 0, 1'000'000'000), Bno086Shtp::PollStatus::SensorEvent);
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].stamp_ns, 999'700'000);
}

TEST(Bno086Shtp, timestampRebaseAdjustsLaterRecordsInSamePayload)
{
  FakeTransport transport;
  Bno086Shtp shtp(transport);

  std::vector<std::uint8_t> payload;
  AppendBaseTimestamp(payload, 10);
  AppendSensorReport(payload, ReportId::Accelerometer, 1);
  AppendTimestampRebase(payload, 1);
  AppendSensorReport(payload, ReportId::Accelerometer, 2);
  transport.PushPacket(payload);

  std::vector<TimestampedSensorEvent> events;
  ASSERT_EQ(shtp.Poll(events, 0, 1'000'000'000), Bno086Shtp::PollStatus::SensorEvent);
  ASSERT_EQ(events.size(), 2U);
  EXPECT_EQ(events[0].stamp_ns, 1'000'000'000);
  EXPECT_EQ(events[1].stamp_ns, 1'000'100'000);
}
} // namespace OASIS::IMU::BNO086
