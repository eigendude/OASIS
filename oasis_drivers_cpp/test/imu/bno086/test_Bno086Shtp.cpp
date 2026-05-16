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
    packet.raw_length = static_cast<std::uint16_t>(payload.size() + kShtpHeaderBytes);
    packet.packet_length = static_cast<std::uint16_t>(payload.size() + kShtpHeaderBytes);
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

TEST(Bno086Transport, headerContinuationBitIsMaskedFromPacketLength)
{
  std::array<std::uint8_t, 4> header{};
  header[0] = 14;
  header[1] = 0x80;
  header[2] = kReportChannel;
  header[3] = 42;

  Bno086ShtpPacket packetHeader;
  ASSERT_TRUE(Bno086Transport::ParseShtpHeaderBytes(header, packetHeader));
  EXPECT_EQ(packetHeader.raw_length, 0x800eU);
  EXPECT_EQ(packetHeader.packet_length, 14U);
  EXPECT_TRUE(packetHeader.continuation);
  EXPECT_EQ(packetHeader.channel, kReportChannel);
  EXPECT_EQ(packetHeader.sequence, 42);
}

TEST(Bno086Transport, closedReadPacketUpdatesDurationDiagnostics)
{
  Bno086Transport transport;
  Bno086ShtpPacket packet;

  EXPECT_FALSE(transport.ReadPacket(packet, 5));

  const Bno086TransportDiagnostics& diagnostics = transport.GetDiagnostics();
  EXPECT_EQ(diagnostics.transport_read_calls, 1U);
  EXPECT_GE(diagnostics.latest_transport_read_duration_ms, 0.0);
  EXPECT_GE(diagnostics.max_transport_read_duration_ms,
            diagnostics.latest_transport_read_duration_ms);
  EXPECT_EQ(diagnostics.transport_read_over_timeout_count, 0U);
  EXPECT_EQ(diagnostics.transport_header_read_calls, 0U);
  EXPECT_EQ(diagnostics.transport_packet_read_calls, 0U);
  EXPECT_EQ(diagnostics.read_packet_attempts, 0U);
  EXPECT_EQ(diagnostics.transaction_failures, 0U);
  EXPECT_EQ(diagnostics.latest_transaction_bytes, 0U);
}

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
  EXPECT_EQ(events[0].stamp_ns, 999'000'000);
  EXPECT_EQ(events[1].stamp_ns, 999'000'000);
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
  packet.raw_length = 0x8004U;
  packet.packet_length = 4;
  packet.channel = kReportChannel;
  packet.sequence = 9;
  packet.continuation = true;
  transport.PushPacket(packet);

  PollOnce(shtp, 1'000'000'000);

  const ShtpDiagnostics& diagnostics = shtp.GetDiagnostics();
  EXPECT_EQ(diagnostics.continuation_packets_reset, 1U);
  EXPECT_EQ(diagnostics.latest_continuation_reset_channel, kReportChannel);
  EXPECT_EQ(diagnostics.latest_continuation_reset_incoming_bytes, 0U);
  EXPECT_EQ(diagnostics.latest_continuation_reset_reason, ContinuationResetReason::EmptyPayload);
  EXPECT_EQ(diagnostics.latest_continuation_reset_raw_length, 0x8004U);
  EXPECT_EQ(diagnostics.latest_continuation_reset_packet_length, 4U);
  EXPECT_EQ(diagnostics.latest_continuation_reset_sequence, 9U);
  EXPECT_FALSE(diagnostics.latest_continuation_reset_had_active_buffer);
}

TEST(Bno086Shtp, orphanContinuationWithValidSensorPayloadIsDecoded)
{
  FakeTransport transport;
  Bno086Shtp shtp(transport);

  Bno086ShtpPacket packet;
  packet.raw_length = 0x800eU;
  packet.packet_length = 14;
  packet.channel = kReportChannel;
  packet.continuation = true;
  packet.payload = PayloadWithOneReport(ReportId::Accelerometer, 3);
  transport.PushPacket(packet);

  std::vector<TimestampedSensorEvent> events;
  EXPECT_EQ(shtp.Poll(events, 0, 1'000'000'000), Bno086Shtp::PollStatus::SensorEvent);
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].event.report_id, ReportId::Accelerometer);
  EXPECT_EQ(shtp.GetDiagnostics().continuation_without_active_buffer, 1U);
  EXPECT_EQ(shtp.GetDiagnostics().continuation_flag_on_decodable_payload, 1U);
  EXPECT_EQ(shtp.GetDiagnostics().continuation_flag_on_sensor_payload, 1U);
  EXPECT_EQ(shtp.GetDiagnostics().orphan_continuation_decoded, 1U);
  EXPECT_EQ(shtp.GetDiagnostics().orphan_continuation_discarded, 0U);
}

TEST(Bno086Shtp, continuationFlagOnControlFeatureResponseIsHandledNormally)
{
  FakeTransport transport;
  Bno086Shtp shtp(transport);

  std::vector<std::uint8_t> payload(17, 0);
  payload[0] = kShtpGetFeatureResponse;
  payload[1] = static_cast<std::uint8_t>(ReportId::Gravity);
  payload[5] = 0x10;
  payload[6] = 0x27;

  Bno086ShtpPacket packet;
  packet.raw_length = static_cast<std::uint16_t>(0x8000U | (payload.size() + kShtpHeaderBytes));
  packet.packet_length = static_cast<std::uint16_t>(payload.size() + kShtpHeaderBytes);
  packet.channel = 2;
  packet.sequence = 79;
  packet.continuation = true;
  packet.payload = payload;
  transport.PushPacket(packet);

  std::vector<TimestampedSensorEvent> events;
  EXPECT_EQ(shtp.Poll(events, 0, 1'000'000'000), Bno086Shtp::PollStatus::PacketHandled);
  EXPECT_TRUE(events.empty());

  const std::vector<FeatureResponse> featureResponses = shtp.TakeFeatureResponses();
  ASSERT_EQ(featureResponses.size(), 1U);
  EXPECT_EQ(featureResponses[0].report_id, ReportId::Gravity);
  EXPECT_EQ(featureResponses[0].report_interval_us, 10'000U);
  EXPECT_EQ(shtp.GetDiagnostics().continuation_packets_reset, 0U);
  EXPECT_EQ(shtp.GetDiagnostics().continuation_flag_on_control_payload, 1U);
  EXPECT_EQ(shtp.GetDiagnostics().continuation_flag_on_decodable_payload, 1U);
}

TEST(Bno086Shtp, validSensorPayloadThatLooksSomewhatLikeHeaderIsNotReset)
{
  FakeTransport transport;
  Bno086Shtp shtp(transport);

  std::vector<std::uint8_t> payload;
  AppendSensorReport(payload, ReportId::Accelerometer, 1);
  payload[2] = 0;
  payload[3] = 0;

  Bno086ShtpPacket packet;
  packet.raw_length = 0x800eU;
  packet.packet_length = 14;
  packet.channel = kReportChannel;
  packet.continuation = true;
  packet.payload = payload;
  transport.PushPacket(packet);

  std::vector<TimestampedSensorEvent> events;
  EXPECT_EQ(shtp.Poll(events, 0, 1'000'000'000), Bno086Shtp::PollStatus::SensorEvent);
  EXPECT_EQ(shtp.GetDiagnostics().continuation_packets_reset, 0U);
  EXPECT_EQ(shtp.GetDiagnostics().embedded_shtp_header_payloads, 0U);
}

TEST(Bno086Shtp, realEmbeddedHeaderLookingOrphanContinuationIsDiscarded)
{
  FakeTransport transport;
  Bno086Shtp shtp(transport);

  Bno086ShtpPacket packet;
  packet.raw_length = 0x8010U;
  packet.packet_length = 16;
  packet.channel = kReportChannel;
  packet.sequence = 12;
  packet.continuation = true;
  packet.payload = {8, 0, kReportChannel, 44, 0xAA, 0xBB, 0xCC, 0xDD};
  transport.PushPacket(packet);

  PollOnce(shtp, 1'000'000'000);

  const ShtpDiagnostics& diagnostics = shtp.GetDiagnostics();
  EXPECT_EQ(diagnostics.embedded_shtp_header_payloads, 1U);
  EXPECT_EQ(diagnostics.orphan_continuation_discarded, 1U);
  EXPECT_EQ(diagnostics.continuation_packets_reset, 1U);
  EXPECT_EQ(diagnostics.latest_continuation_reset_reason,
            ContinuationResetReason::EmbeddedShtpHeader);
  EXPECT_EQ(diagnostics.latest_continuation_reset_payload_prefix_size, 8U);
  EXPECT_EQ(diagnostics.latest_continuation_reset_payload_prefix[0], 8U);
}

TEST(Bno086Shtp, continuationStartAndFinishDecodesCombinedPayload)
{
  FakeTransport transport;
  Bno086Shtp shtp(transport);

  std::vector<std::uint8_t> payload = PayloadWithOneReport(ReportId::Accelerometer, 5);

  Bno086ShtpPacket first;
  first.raw_length = 0x8007U;
  first.packet_length = 7;
  first.channel = kReportChannel;
  first.continuation = true;
  first.payload.assign(payload.begin(), payload.begin() + 3);
  transport.PushPacket(first);

  Bno086ShtpPacket second;
  second.raw_length = static_cast<std::uint16_t>(payload.size() - 3 + kShtpHeaderBytes);
  second.packet_length = second.raw_length;
  second.channel = kReportChannel;
  second.payload.assign(payload.begin() + 3, payload.end());
  transport.PushPacket(second);

  std::vector<TimestampedSensorEvent> events;
  EXPECT_EQ(shtp.Poll(events, 0, 1'000'000'000), Bno086Shtp::PollStatus::PacketHandled);
  EXPECT_TRUE(events.empty());

  EXPECT_EQ(shtp.Poll(events, 0, 1'000'100'000), Bno086Shtp::PollStatus::SensorEvent);
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].event.sequence, 5);
  EXPECT_EQ(shtp.GetDiagnostics().continuation_packets_started, 1U);
  EXPECT_EQ(shtp.GetDiagnostics().continuation_packets_completed, 1U);
  EXPECT_EQ(shtp.GetDiagnostics().active_fragment_buffers_started, 1U);
  EXPECT_EQ(shtp.GetDiagnostics().active_fragment_buffers_completed, 1U);
}

TEST(Bno086Shtp, packetSequenceGapsAreTrackedPerChannel)
{
  FakeTransport transport;
  Bno086Shtp shtp(transport);

  Bno086ShtpPacket first;
  first.raw_length = 14;
  first.packet_length = 14;
  first.channel = kReportChannel;
  first.sequence = 10;
  first.payload = PayloadWithOneReport(ReportId::Accelerometer, 1);
  transport.PushPacket(first);

  Bno086ShtpPacket second = first;
  second.sequence = 13;
  second.payload = PayloadWithOneReport(ReportId::Accelerometer, 2);
  transport.PushPacket(second);

  PollOnce(shtp, 1'000'000'000);
  PollOnce(shtp, 1'000'100'000);

  EXPECT_EQ(shtp.GetDiagnostics().packet_sequence_gaps_by_channel[kReportChannel], 1U);
  EXPECT_EQ(shtp.GetDiagnostics().packet_sequence_gap_max_by_channel[kReportChannel], 3U);
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
  EXPECT_EQ(events[0].stamp_ns, 999'300'000);
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
  EXPECT_EQ(events[0].stamp_ns, 999'000'000);
  EXPECT_EQ(events[1].stamp_ns, 998'900'000);
}
} // namespace OASIS::IMU::BNO086
