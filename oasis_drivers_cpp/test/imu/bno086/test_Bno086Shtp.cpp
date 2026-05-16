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
    PushPacket(packet);
  }

  void PushPacket(const Bno086ShtpPacket& packet) { m_packets.emplace_back(packet); }

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

std::vector<std::uint8_t> PayloadWithOneReport(ReportId report_id, std::uint8_t sequence)
{
  std::vector<std::uint8_t> payload;
  AppendBaseTimestamp(payload, 10U + sequence);
  AppendSensorReport(payload, report_id, sequence);
  return payload;
}

void PollOnce(Bno086Shtp& shtp, std::int64_t packet_host_stamp_ns)
{
  std::vector<TimestampedSensorEvent> events;
  static_cast<void>(shtp.Poll(events, 0, packet_host_stamp_ns));
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

TEST(Bno086Shtp, multiReportPayloadUpdatesDecodeDiagnostics)
{
  FakeTransport transport;
  Bno086Shtp shtp(transport);

  std::vector<std::uint8_t> payload;
  AppendBaseTimestamp(payload, 10);
  AppendSensorReport(payload, ReportId::Accelerometer, 1);
  AppendSensorReport(payload, ReportId::GyroscopeCalibrated, 2);
  AppendSensorReport(payload, ReportId::RotationVector, 3);
  transport.PushPacket(payload);

  PollOnce(shtp, 1'000'000'000);

  const ShtpDiagnostics& diagnostics = shtp.GetDiagnostics();
  EXPECT_EQ(diagnostics.packets_read, 1U);
  EXPECT_EQ(diagnostics.sensor_packets_read, 1U);
  EXPECT_EQ(diagnostics.sensor_events_decoded, 3U);
  EXPECT_EQ(diagnostics.latest_events_per_packet, 3U);
  EXPECT_EQ(diagnostics.max_events_per_packet, 3U);
  EXPECT_GE(diagnostics.max_packet_payload_bytes, payload.size());
}

TEST(Bno086Shtp, perReportDecodedCountsIncrement)
{
  FakeTransport transport;
  Bno086Shtp shtp(transport);

  std::vector<std::uint8_t> payload;
  AppendBaseTimestamp(payload, 10);
  AppendSensorReport(payload, ReportId::Accelerometer, 1);
  AppendSensorReport(payload, ReportId::GyroscopeCalibrated, 2);
  AppendSensorReport(payload, ReportId::RotationVector, 3);
  AppendSensorReport(payload, ReportId::LinearAcceleration, 4);
  AppendSensorReport(payload, ReportId::Gravity, 5);
  transport.PushPacket(payload);

  PollOnce(shtp, 1'000'000'000);

  const ShtpDiagnostics& diagnostics = shtp.GetDiagnostics();
  EXPECT_EQ(diagnostics.decoded_report_counts[static_cast<std::uint8_t>(ReportId::Accelerometer)],
            1U);
  EXPECT_EQ(
      diagnostics.decoded_report_counts[static_cast<std::uint8_t>(ReportId::GyroscopeCalibrated)],
      1U);
  EXPECT_EQ(diagnostics.decoded_report_counts[static_cast<std::uint8_t>(ReportId::RotationVector)],
            1U);
  EXPECT_EQ(
      diagnostics.decoded_report_counts[static_cast<std::uint8_t>(ReportId::LinearAcceleration)],
      1U);
  EXPECT_EQ(diagnostics.decoded_report_counts[static_cast<std::uint8_t>(ReportId::Gravity)], 1U);
}

TEST(Bno086Shtp, sequenceDeltaOneDoesNotCountGap)
{
  FakeTransport transport;
  Bno086Shtp shtp(transport);

  transport.PushPacket(PayloadWithOneReport(ReportId::Accelerometer, 1));
  transport.PushPacket(PayloadWithOneReport(ReportId::Accelerometer, 2));

  PollOnce(shtp, 1'000'000'000);
  PollOnce(shtp, 1'000'100'000);

  const ReportSequenceDiagnostics& sequenceDiagnostics =
      shtp.GetDiagnostics().report_sequence[static_cast<std::uint8_t>(ReportId::Accelerometer)];
  EXPECT_EQ(sequenceDiagnostics.gap_count, 0U);
  EXPECT_EQ(sequenceDiagnostics.gap_max, 0U);
}

TEST(Bno086Shtp, sequenceDeltaGreaterThanOneCountsGap)
{
  FakeTransport transport;
  Bno086Shtp shtp(transport);

  transport.PushPacket(PayloadWithOneReport(ReportId::Accelerometer, 1));
  transport.PushPacket(PayloadWithOneReport(ReportId::Accelerometer, 4));

  PollOnce(shtp, 1'000'000'000);
  PollOnce(shtp, 1'000'300'000);

  const ReportSequenceDiagnostics& sequenceDiagnostics =
      shtp.GetDiagnostics().report_sequence[static_cast<std::uint8_t>(ReportId::Accelerometer)];
  EXPECT_EQ(sequenceDiagnostics.gap_count, 1U);
  EXPECT_EQ(sequenceDiagnostics.gap_max, 3U);
}

TEST(Bno086Shtp, uint8SequenceWrapDoesNotCountGap)
{
  FakeTransport transport;
  Bno086Shtp shtp(transport);

  transport.PushPacket(PayloadWithOneReport(ReportId::Accelerometer, 255));
  transport.PushPacket(PayloadWithOneReport(ReportId::Accelerometer, 0));

  PollOnce(shtp, 1'000'000'000);
  PollOnce(shtp, 1'000'100'000);

  const ReportSequenceDiagnostics& sequenceDiagnostics =
      shtp.GetDiagnostics().report_sequence[static_cast<std::uint8_t>(ReportId::Accelerometer)];
  EXPECT_EQ(sequenceDiagnostics.gap_count, 0U);
  EXPECT_EQ(sequenceDiagnostics.gap_max, 0U);
}

TEST(Bno086Shtp, duplicateSequenceIncrementsDuplicateCounter)
{
  FakeTransport transport;
  Bno086Shtp shtp(transport);

  transport.PushPacket(PayloadWithOneReport(ReportId::Accelerometer, 7));
  transport.PushPacket(PayloadWithOneReport(ReportId::Accelerometer, 7));

  PollOnce(shtp, 1'000'000'000);
  PollOnce(shtp, 1'000'100'000);

  const ReportSequenceDiagnostics& sequenceDiagnostics =
      shtp.GetDiagnostics().report_sequence[static_cast<std::uint8_t>(ReportId::Accelerometer)];
  EXPECT_EQ(sequenceDiagnostics.duplicate_sequence_count, 1U);
}

TEST(Bno086Shtp, malformedSensorPayloadIncrementsDecodeDiagnostics)
{
  FakeTransport transport;
  Bno086Shtp shtp(transport);

  std::vector<std::uint8_t> payload;
  payload.emplace_back(static_cast<std::uint8_t>(ReportId::Accelerometer));
  payload.emplace_back(1);
  payload.emplace_back(0);
  transport.PushPacket(payload);

  PollOnce(shtp, 1'000'000'000);

  EXPECT_EQ(shtp.GetDiagnostics().malformed_sensor_payloads, 1U);
  EXPECT_EQ(shtp.GetDiagnostics().decode_errors, 1U);
}

TEST(Bno086Shtp, continuationResetIncrementsDiagnostics)
{
  FakeTransport transport;
  Bno086Shtp shtp(transport);

  Bno086ShtpPacket packet;
  packet.channel = kReportChannel;
  packet.continuation = true;
  transport.PushPacket(packet);

  PollOnce(shtp, 1'000'000'000);

  const ShtpDiagnostics& diagnostics = shtp.GetDiagnostics();
  EXPECT_EQ(diagnostics.continuation_packets_reset, 1U);
  EXPECT_EQ(diagnostics.latest_continuation_reset_channel, kReportChannel);
  EXPECT_EQ(diagnostics.latest_continuation_reset_incoming_bytes, 0U);
  EXPECT_EQ(diagnostics.latest_continuation_reset_reason, ContinuationResetReason::EmptyPayload);
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
