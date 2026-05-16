/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/Bno086Shtp.hpp"

#include <algorithm>
#include <cmath>

namespace OASIS::IMU::BNO086
{
namespace
{
constexpr std::uint8_t kChannelCommand = 0;
constexpr std::uint8_t kChannelExecutable = 1;
constexpr std::uint8_t kChannelControl = 2;
constexpr std::uint8_t kChannelReports = 3;
constexpr std::uint8_t kChannelWakeReports = 4;
constexpr std::uint8_t kChannelGyroRotationVector = 5;

constexpr std::uint8_t kSensorFlushCompleted = 0xEF;
constexpr std::size_t kBaseTimestampHeaderBytes = 5;
constexpr std::size_t kSensorCommonHeaderBytes = 4;
constexpr std::size_t kFeatureResponseBytes = 17;
constexpr std::size_t kGyroIntegratedRotationVectorPayloadBytes = 14;
constexpr std::size_t kMaxContinuationBytes = 4096;

// Target report interval for the 100 Hz imu_gravity cadence
constexpr std::uint32_t kTargetReportIntervalUs = 10'000;

constexpr std::array<ReportId, 5> kConfiguredReports = {
    ReportId::RotationVector,
    ReportId::GyroscopeCalibrated,
    ReportId::LinearAcceleration,
    ReportId::Accelerometer,
    ReportId::Gravity,
};

bool LooksLikeEmbeddedShtpHeader(const std::vector<std::uint8_t>& payload)
{
  if (payload.size() < kShtpHeaderBytes)
    return false;

  const std::uint16_t rawLength =
      static_cast<std::uint16_t>(payload[0] | (static_cast<std::uint16_t>(payload[1]) << 8));
  const std::size_t packetLength = static_cast<std::size_t>(rawLength & 0x7FFFU);
  const std::uint8_t channel = payload[2];

  if (packetLength < kShtpHeaderBytes || packetLength > 1024)
    return false;

  if (channel > kChannelGyroRotationVector)
    return false;

  return packetLength <= payload.size();
}
} // namespace

Bno086Shtp::Bno086Shtp(Bno086Transport& transport) : m_transport(transport)
{
}

bool Bno086Shtp::Configure(const Bno086ShtpConfig& config)
{
  m_config = config;
  m_config.report_rate_hz = std::max(m_config.report_rate_hz, 1.0);

  // Set Feature expects report interval in microseconds
  // This configures BNO086 sensor timing, not ROS publish timing
  m_reportIntervalUs = ToReportIntervalUs(m_config.report_rate_hz);
  m_featureConfigurations.clear();
  m_featureResponses.clear();

  for (const ReportId reportId : kConfiguredReports)
  {
    FeatureConfiguration featureConfiguration;
    featureConfiguration.report_id = reportId;
    featureConfiguration.requested_interval_us =
        RequestedIntervalForReport(reportId, m_reportIntervalUs);
    m_featureConfigurations.emplace_back(featureConfiguration);
  }

  m_configRequested = true;
  m_startupStatus.set_feature_sent = false;
  m_lastConfigAttempt = {};

  return SendSetFeatureCommands();
}

Bno086Shtp::PollStatus Bno086Shtp::Poll(std::vector<TimestampedSensorEvent>& events,
                                        int timeout_ms,
                                        std::int64_t packet_host_stamp_ns)
{
  events.clear();

  Bno086ShtpPacket packet;
  if (!m_transport.ReadPacket(packet, timeout_ms))
    return PollStatus::Timeout;

  RecordPacketRead(packet);

  if (!DecodePacket(packet, events, packet_host_stamp_ns))
    return PollStatus::TransportError;

  if (!events.empty())
    return PollStatus::SensorEvent;

  return PollStatus::PacketHandled;
}

const Bno086Shtp::StartupStatus& Bno086Shtp::GetStartupStatus() const
{
  return m_startupStatus;
}

const ShtpDiagnostics& Bno086Shtp::GetDiagnostics() const
{
  return m_diagnostics;
}

const TimestampReconstructionDiagnostics& Bno086Shtp::GetTimestampReconstructionDiagnostics() const
{
  return m_timestampReconstructor.GetDiagnostics();
}

const std::vector<FeatureConfiguration>& Bno086Shtp::GetFeatureConfigurations() const
{
  return m_featureConfigurations;
}

std::vector<FeatureResponse> Bno086Shtp::TakeFeatureResponses()
{
  std::vector<FeatureResponse> featureResponses;
  featureResponses.swap(m_featureResponses);
  return featureResponses;
}

bool Bno086Shtp::SendSetFeatureCommands()
{
  if (!m_configRequested)
    return true;

  bool allOk = true;
  for (const FeatureConfiguration& featureConfiguration : m_featureConfigurations)
  {
    if (!ConfigureFeature(featureConfiguration.report_id,
                          featureConfiguration.requested_interval_us))
    {
      allOk = false;
      continue;
    }

    if (!RequestFeature(featureConfiguration.report_id))
      allOk = false;
  }

  m_startupStatus.set_feature_sent = allOk;
  m_lastConfigAttempt = std::chrono::steady_clock::now();
  return allOk;
}

bool Bno086Shtp::ConfigureFeature(ReportId report_id, std::uint32_t interval_us)
{
  std::vector<std::uint8_t> payload(17, 0);
  payload[0] = kShtpSetFeatureCommand;
  payload[1] = static_cast<std::uint8_t>(report_id);

  payload[5] = static_cast<std::uint8_t>(interval_us & 0xFF);
  payload[6] = static_cast<std::uint8_t>((interval_us >> 8) & 0xFF);
  payload[7] = static_cast<std::uint8_t>((interval_us >> 16) & 0xFF);
  payload[8] = static_cast<std::uint8_t>((interval_us >> 24) & 0xFF);

  return m_transport.WritePacket(kChannelControl, payload);
}

bool Bno086Shtp::RequestFeature(ReportId report_id)
{
  std::vector<std::uint8_t> payload(2, 0);
  payload[0] = kShtpGetFeatureRequest;
  payload[1] = static_cast<std::uint8_t>(report_id);

  return m_transport.WritePacket(kChannelControl, payload);
}

bool Bno086Shtp::DecodePacket(const Bno086ShtpPacket& packet,
                              std::vector<TimestampedSensorEvent>& events,
                              std::int64_t packet_host_stamp_ns)
{
  MarkCommunicationEstablished();

  const std::size_t eventsBefore = events.size();
  m_diagnostics.latest_events_per_packet = 0;

  Bno086ShtpPacket normalizedPacket = packet;
  bool shouldDecodeSensorPayload = IsSensorChannel(packet.channel);
  bool payloadCanContinue = packet.continuation;

  const bool embeddedHeader = LooksLikeEmbeddedShtpHeader(packet.payload);
  if (embeddedHeader)
    ++m_diagnostics.embedded_shtp_header_payloads;

  if (packet.channel < m_continuationPayloads.size())
  {
    const bool active = m_continuationActive[packet.channel];
    const std::size_t activeBytes = m_continuationPayloads[packet.channel].size();

    if (packet.continuation)
    {
      if (activeBytes == 0)
        ++m_diagnostics.continuation_with_zero_accumulated_bytes;

      if (!active)
      {
        ++m_diagnostics.continuation_without_active_buffer;

        if (IsSensorChannel(packet.channel) && IsValidSensorPayloadStart(packet.payload))
        {
          ++m_diagnostics.orphan_continuation_decoded;
          shouldDecodeSensorPayload = true;
          normalizedPacket.payload = packet.payload;
        }
        else
        {
          ++m_diagnostics.orphan_continuation_discarded;
          const ContinuationResetReason resetReason =
              packet.payload.empty()
                  ? ContinuationResetReason::EmptyPayload
                  : (embeddedHeader ? ContinuationResetReason::EmbeddedShtpHeader
                                    : ContinuationResetReason::OrphanContinuation);
          RecordContinuationReset(packet.channel, packet.sequence, packet.raw_length,
                                  packet.packet_length, activeBytes, packet.payload.size(),
                                  resetReason, active, packet.payload);
          MaybeSendDeferredConfiguration();
          return true;
        }
      }
      else
      {
        auto& continuationPayload = m_continuationPayloads[packet.channel];
        normalizedPacket.payload = continuationPayload;
        normalizedPacket.payload.insert(normalizedPacket.payload.end(), packet.payload.begin(),
                                        packet.payload.end());
        shouldDecodeSensorPayload = IsSensorChannel(packet.channel);
      }
    }
    else if (active)
    {
      auto& continuationPayload = m_continuationPayloads[packet.channel];
      ++m_diagnostics.continuation_packets_completed;
      normalizedPacket.payload = continuationPayload;
      normalizedPacket.payload.insert(normalizedPacket.payload.end(), packet.payload.begin(),
                                      packet.payload.end());
      m_continuationActive[packet.channel] = false;
      continuationPayload.clear();
      shouldDecodeSensorPayload = IsSensorChannel(packet.channel);
      payloadCanContinue = false;
    }

    if (!IsSensorChannel(packet.channel))
    {
      const ContinuationValidation validation = ValidateContinuationFragment(packet);
      if (!validation.valid)
      {
        RecordContinuationReset(packet.channel, packet.sequence, packet.raw_length,
                                packet.packet_length, activeBytes, packet.payload.size(),
                                validation.reset_reason, active, packet.payload);

        if (validation.flush_buffer)
          ClearContinuationState(packet.channel);

        return true;
      }

      if (validation.keep_buffering)
      {
        auto& continuationPayload = m_continuationPayloads[packet.channel];
        if (!m_continuationActive[packet.channel])
          ++m_diagnostics.continuation_packets_started;

        m_continuationActive[packet.channel] = true;
        continuationPayload.insert(continuationPayload.end(), packet.payload.begin(),
                                   packet.payload.end());
        return true;
      }
    }
  }

  if (normalizedPacket.channel < kChannelReports)
  {
    if (normalizedPacket.channel == kChannelControl)
      DecodeControlPayload(normalizedPacket.payload);

    MaybeSendDeferredConfiguration();
    return true;
  }

  if (shouldDecodeSensorPayload)
  {
    const SensorDecodeResult decoded =
        DecodeSensorPayload(normalizedPacket.payload, normalizedPacket.channel, events,
                            packet_host_stamp_ns, payloadCanContinue);

    if (decoded.malformed_payload)
      ++m_diagnostics.malformed_sensor_payloads;

    if (decoded.decode_error)
      ++m_diagnostics.decode_errors;

    const std::size_t eventsThisPacket = events.size() - eventsBefore;
    m_diagnostics.latest_events_per_packet = static_cast<std::uint32_t>(eventsThisPacket);
    m_diagnostics.max_events_per_packet =
        std::max(m_diagnostics.max_events_per_packet, static_cast<std::uint32_t>(eventsThisPacket));

    if (packet.channel < m_continuationPayloads.size())
    {
      if (decoded.trailing_bytes > 0)
      {
        auto& continuationPayload = m_continuationPayloads[packet.channel];
        if (!m_continuationActive[packet.channel])
          ++m_diagnostics.continuation_packets_started;

        continuationPayload.assign(normalizedPacket.payload.end() -
                                       static_cast<std::ptrdiff_t>(decoded.trailing_bytes),
                                   normalizedPacket.payload.end());
        m_continuationActive[packet.channel] = true;
      }
      else
      {
        m_continuationActive[packet.channel] = false;
        m_continuationPayloads[packet.channel].clear();
      }
    }

    MaybeSendDeferredConfiguration();
    return true;
  }

  MaybeSendDeferredConfiguration();
  return true;
}

void Bno086Shtp::DecodeControlPayload(const std::vector<std::uint8_t>& payload)
{
  std::size_t offset = 0;

  while (offset < payload.size())
  {
    const std::uint8_t reportCode = payload[offset];

    if (reportCode != kShtpGetFeatureResponse)
    {
      ++offset;
      continue;
    }

    if (offset + kFeatureResponseBytes > payload.size())
      return;

    const std::uint8_t featureReportId = payload[offset + 1];
    if (IsConfiguredReport(featureReportId))
    {
      FeatureResponse featureResponse;
      featureResponse.report_id = static_cast<ReportId>(featureReportId);
      featureResponse.feature_flags = payload[offset + 2];
      featureResponse.change_sensitivity = static_cast<std::uint16_t>(payload[offset + 3]) |
                                           static_cast<std::uint16_t>(payload[offset + 4] << 8);
      featureResponse.report_interval_us = ReadU32(payload, offset + 5);
      featureResponse.batch_interval_us = ReadU32(payload, offset + 9);
      featureResponse.sensor_specific_config = ReadU32(payload, offset + 13);
      m_featureResponses.emplace_back(featureResponse);
    }

    offset += kFeatureResponseBytes;
  }
}

Bno086Shtp::SensorDecodeResult Bno086Shtp::DecodeSensorPayload(
    const std::vector<std::uint8_t>& payload,
    std::uint8_t channel,
    std::vector<TimestampedSensorEvent>& events,
    std::int64_t packet_host_stamp_ns,
    bool payload_can_continue)
{
  SensorDecodeResult result;

  if (payload.empty())
    return result;

  if (channel == kChannelGyroRotationVector)
  {
    if (payload.size() < kGyroIntegratedRotationVectorPayloadBytes)
      result.trailing_bytes = payload.size();

    return result;
  }

  std::size_t offset = 0;
  std::optional<std::uint32_t> baseTimestampUs;

  while (offset < payload.size())
  {
    const std::uint8_t reportCode = payload[offset];

    if (reportCode == kShtpReportBaseTimestamp)
    {
      if (offset + kBaseTimestampHeaderBytes > payload.size())
      {
        result.trailing_bytes = payload.size() - offset;
        result.malformed_payload = !payload_can_continue;
        result.decode_error = !payload_can_continue;
        break;
      }

      baseTimestampUs = ReadU32(payload, offset + 1) * 100U;
      offset += kBaseTimestampHeaderBytes;
      continue;
    }

    if (reportCode == kShtpReportTimestampRebase)
    {
      if (offset + kBaseTimestampHeaderBytes > payload.size())
      {
        result.trailing_bytes = payload.size() - offset;
        result.malformed_payload = !payload_can_continue;
        result.decode_error = !payload_can_continue;
        break;
      }

      if (baseTimestampUs.has_value())
        *baseTimestampUs += ReadU32(payload, offset + 1) * 100U;

      offset += kBaseTimestampHeaderBytes;
      continue;
    }

    if (reportCode == kSensorFlushCompleted)
    {
      constexpr std::size_t kFlushCompletedBytes = 2;
      if (offset + kFlushCompletedBytes > payload.size())
      {
        result.trailing_bytes = payload.size() - offset;
        result.malformed_payload = !payload_can_continue;
        result.decode_error = !payload_can_continue;
        break;
      }

      offset += kFlushCompletedBytes;
      continue;
    }

    const std::optional<std::size_t> recordBytes = SensorRecordBytes(reportCode);
    if (!recordBytes.has_value())
    {
      ++m_diagnostics.unknown_sensor_reports;
      ++offset;
      continue;
    }

    if (offset + *recordBytes > payload.size())
    {
      result.trailing_bytes = payload.size() - offset;
      result.malformed_payload = !payload_can_continue;
      result.decode_error = !payload_can_continue;
      break;
    }

    SensorEvent decodedEvent;
    std::size_t bytesConsumed = 0;
    if (!DecodeSingleSensorReport(payload, offset, baseTimestampUs, decodedEvent, bytesConsumed))
    {
      result.decode_error = true;
      offset += *recordBytes;
      continue;
    }

    RecordDecodedSensorEvent(decodedEvent);

    TimestampedSensorEvent timestampedEvent;
    timestampedEvent.event = decodedEvent;
    timestampedEvent.stamp_ns =
        m_timestampReconstructor.Reconstruct(decodedEvent, packet_host_stamp_ns);
    events.emplace_back(timestampedEvent);

    offset += bytesConsumed;
  }

  return result;
}

bool Bno086Shtp::DecodeSingleSensorReport(const std::vector<std::uint8_t>& payload,
                                          std::size_t report_offset,
                                          std::optional<std::uint32_t> base_timestamp_us,
                                          SensorEvent& event,
                                          std::size_t& bytes_consumed) const
{
  bytes_consumed = 0;

  if (report_offset >= payload.size())
    return false;

  const std::uint8_t reportCode = payload[report_offset];
  if (!IsTrackedReport(reportCode))
    return false;

  const ReportId reportId = static_cast<ReportId>(reportCode);
  const std::size_t payloadBytes = SensorPayloadBytes(reportId);
  const std::size_t recordBytes = kSensorCommonHeaderBytes + payloadBytes;
  if (report_offset + recordBytes > payload.size())
    return false;

  event = SensorEvent{};
  event.report_id = reportId;
  event.sequence = payload[report_offset + 1];
  event.status = payload[report_offset + 2];
  event.accuracy = static_cast<std::uint8_t>(event.status & 0x03U);

  // SH-2 combines accuracy in bits 1:0 with the upper six bits of the
  // 14-bit delay field in bits 7:2. Byte 3 contributes the lower eight bits.
  const std::uint16_t delayTicks =
      static_cast<std::uint16_t>((payload[report_offset + 2] >> 2) << 8) |
      static_cast<std::uint16_t>(payload[report_offset + 3]);
  event.delay_us = delayTicks * 100U;
  event.has_delay = true;

  const std::size_t payloadOffset = report_offset + kSensorCommonHeaderBytes;
  for (std::size_t valueIndex = 0; valueIndex < payloadBytes / 2; ++valueIndex)
    event.values[valueIndex] = ReadS16(payload, payloadOffset + valueIndex * 2);

  if (base_timestamp_us.has_value())
  {
    event.base_timestamp_us = *base_timestamp_us;
    event.has_base_timestamp = true;
  }

  bytes_consumed = recordBytes;
  return true;
}

void Bno086Shtp::MaybeSendDeferredConfiguration()
{
  if (!m_configRequested)
    return;

  if (!m_startupStatus.communication_established)
    return;

  if (m_startupStatus.set_feature_sent)
    return;

  const auto now = std::chrono::steady_clock::now();
  if (m_lastConfigAttempt.time_since_epoch().count() != 0)
  {
    const auto elapsedMs =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - m_lastConfigAttempt).count();
    if (elapsedMs < 100)
      return;
  }

  SendSetFeatureCommands();
}

void Bno086Shtp::RecordPacketRead(const Bno086ShtpPacket& packet)
{
  ++m_diagnostics.packets_read;

  if (packet.continuation)
    ++m_diagnostics.packets_with_continuation_flag;

  if (IsSensorChannel(packet.channel))
    ++m_diagnostics.sensor_packets_read;

  if (packet.channel == kChannelControl)
    ++m_diagnostics.control_packets_read;

  if (packet.channel == kChannelExecutable)
    ++m_diagnostics.executable_packets_read;

  if (packet.channel == kChannelWakeReports)
    ++m_diagnostics.wake_packets_read;

  m_diagnostics.latest_raw_length = packet.raw_length;
  m_diagnostics.latest_packet_length = packet.packet_length;
  m_diagnostics.latest_continuation_flag = packet.continuation;
  m_diagnostics.latest_channel = packet.channel;
  m_diagnostics.latest_sequence = packet.sequence;
  m_diagnostics.latest_packet_payload_bytes = static_cast<std::uint32_t>(packet.payload.size());
  m_diagnostics.max_packet_payload_bytes = std::max(
      m_diagnostics.max_packet_payload_bytes, static_cast<std::uint32_t>(packet.payload.size()));
}

void Bno086Shtp::RecordDecodedSensorEvent(const SensorEvent& event)
{
  const std::size_t reportIndex =
      static_cast<std::size_t>(static_cast<std::uint8_t>(event.report_id));

  ++m_diagnostics.sensor_events_decoded;
  ++m_diagnostics.decoded_report_counts[reportIndex];

  ReportSequenceDiagnostics& sequenceDiagnostics = m_diagnostics.report_sequence[reportIndex];
  if (sequenceDiagnostics.has_sequence)
  {
    const std::uint8_t delta =
        static_cast<std::uint8_t>(event.sequence - sequenceDiagnostics.last_sequence);

    if (delta == 0)
      ++sequenceDiagnostics.duplicate_sequence_count;
    else if (delta > 1)
    {
      ++sequenceDiagnostics.gap_count;
      sequenceDiagnostics.gap_max = std::max(sequenceDiagnostics.gap_max, delta);
    }
  }

  sequenceDiagnostics.has_sequence = true;
  sequenceDiagnostics.last_sequence = event.sequence;
}

void Bno086Shtp::RecordContinuationReset(std::uint8_t channel,
                                         std::uint8_t sequence,
                                         std::uint16_t raw_length,
                                         std::uint16_t packet_length,
                                         std::size_t accumulated_bytes,
                                         std::size_t incoming_bytes,
                                         ContinuationResetReason reason,
                                         bool had_active_buffer,
                                         const std::vector<std::uint8_t>& payload)
{
  ++m_diagnostics.continuation_packets_reset;
  m_diagnostics.latest_continuation_reset_channel = channel;
  m_diagnostics.latest_continuation_reset_sequence = sequence;
  m_diagnostics.latest_continuation_reset_raw_length = raw_length;
  m_diagnostics.latest_continuation_reset_packet_length = packet_length;
  m_diagnostics.latest_continuation_reset_accumulated_bytes =
      static_cast<std::uint32_t>(accumulated_bytes);
  m_diagnostics.latest_continuation_reset_incoming_bytes =
      static_cast<std::uint32_t>(incoming_bytes);
  m_diagnostics.latest_continuation_reset_reason = reason;
  m_diagnostics.latest_continuation_reset_had_active_buffer = had_active_buffer;
  m_diagnostics.latest_continuation_reset_payload_prefix.fill(0);
  m_diagnostics.latest_continuation_reset_payload_prefix_size =
      static_cast<std::uint8_t>(std::min<std::size_t>(
          payload.size(), m_diagnostics.latest_continuation_reset_payload_prefix.size()));

  for (std::size_t i = 0; i < m_diagnostics.latest_continuation_reset_payload_prefix_size; ++i)
    m_diagnostics.latest_continuation_reset_payload_prefix[i] = payload[i];
}

void Bno086Shtp::MarkCommunicationEstablished()
{
  m_startupStatus.communication_established = true;
}

bool Bno086Shtp::IsSensorChannel(std::uint8_t channel) const
{
  return channel == kChannelReports || channel == kChannelWakeReports ||
         channel == kChannelGyroRotationVector;
}

Bno086Shtp::ContinuationValidation Bno086Shtp::ValidateContinuationFragment(
    const Bno086ShtpPacket& packet) const
{
  ContinuationValidation validation;

  const bool active = m_continuationActive[packet.channel];
  const std::size_t bufferedBytes = m_continuationPayloads[packet.channel].size();

  if (packet.continuation)
  {
    validation.keep_buffering = true;

    if (packet.payload.empty())
    {
      validation.valid = false;
      validation.keep_buffering = false;
      validation.flush_buffer = active;
      validation.reset_reason = ContinuationResetReason::EmptyPayload;
      return validation;
    }

    if (packet.channel == kChannelCommand && packet.payload.size() <= kShtpHeaderBytes)
    {
      validation.valid = false;
      validation.keep_buffering = false;
      validation.flush_buffer = true;
      validation.reset_reason = ContinuationResetReason::CommandHeaderOnly;
      return validation;
    }

    if (LooksLikeEmbeddedShtpHeader(packet.payload))
    {
      validation.valid = false;
      validation.keep_buffering = false;
      validation.flush_buffer = true;
      validation.reset_reason = ContinuationResetReason::ShtpHeaderPrefix;
      return validation;
    }

    if ((bufferedBytes + packet.payload.size()) > kMaxContinuationBytes)
    {
      validation.valid = false;
      validation.keep_buffering = false;
      validation.flush_buffer = true;
      validation.reset_reason = ContinuationResetReason::MaxBytesExceeded;
      return validation;
    }

    return validation;
  }

  if (!active)
    return validation;

  if (packet.payload.empty())
  {
    validation.valid = false;
    validation.flush_buffer = true;
    validation.reset_reason = ContinuationResetReason::EmptyPayload;
    return validation;
  }

  if (LooksLikeEmbeddedShtpHeader(packet.payload))
  {
    validation.valid = false;
    validation.flush_buffer = true;
    validation.reset_reason = ContinuationResetReason::ShtpHeaderPrefix;
    return validation;
  }

  if ((bufferedBytes + packet.payload.size()) > kMaxContinuationBytes)
  {
    validation.valid = false;
    validation.flush_buffer = true;
    validation.reset_reason = ContinuationResetReason::MaxBytesExceeded;
    return validation;
  }

  return validation;
}

bool Bno086Shtp::IsValidSensorPayloadStart(const std::vector<std::uint8_t>& payload) const
{
  if (payload.empty())
    return false;

  const std::uint8_t reportCode = payload.front();
  if (reportCode == kShtpReportBaseTimestamp || reportCode == kShtpReportTimestampRebase ||
      reportCode == kSensorFlushCompleted)
  {
    return true;
  }

  return SensorRecordBytes(reportCode).has_value();
}

void Bno086Shtp::ClearContinuationState(std::uint8_t channel)
{
  if (channel >= m_continuationPayloads.size())
    return;

  m_continuationActive[channel] = false;
  m_continuationPayloads[channel].clear();
}

std::uint32_t Bno086Shtp::ToReportIntervalUs(double rate_hz)
{
  const double clampedHz = std::max(rate_hz, 1.0);
  return static_cast<std::uint32_t>(std::round(1'000'000.0 / clampedHz));
}

std::uint32_t Bno086Shtp::RequestedIntervalForReport(ReportId /*report_id*/,
                                                     std::uint32_t configured_interval_us)
{
  return std::min(configured_interval_us, kTargetReportIntervalUs);
}

std::uint32_t Bno086Shtp::ReadU32(const std::vector<std::uint8_t>& data, std::size_t offset)
{
  if (offset + 3 >= data.size())
    return 0;

  const std::uint32_t b0 = data[offset];
  const std::uint32_t b1 = static_cast<std::uint32_t>(data[offset + 1]) << 8;
  const std::uint32_t b2 = static_cast<std::uint32_t>(data[offset + 2]) << 16;
  const std::uint32_t b3 = static_cast<std::uint32_t>(data[offset + 3]) << 24;
  return b0 | b1 | b2 | b3;
}

std::int16_t Bno086Shtp::ReadS16(const std::vector<std::uint8_t>& data, std::size_t offset)
{
  if (offset + 1 >= data.size())
    return 0;

  const std::uint16_t low = data[offset];
  const std::uint16_t high = static_cast<std::uint16_t>(data[offset + 1]) << 8;
  return static_cast<std::int16_t>(low | high);
}

bool Bno086Shtp::IsTrackedReport(std::uint8_t report_id)
{
  switch (report_id)
  {
    case static_cast<std::uint8_t>(ReportId::Accelerometer):
    case static_cast<std::uint8_t>(ReportId::GyroscopeCalibrated):
    case static_cast<std::uint8_t>(ReportId::LinearAcceleration):
    case static_cast<std::uint8_t>(ReportId::RotationVector):
    case static_cast<std::uint8_t>(ReportId::Gravity):
      return true;
    default:
      break;
  }

  return false;
}

bool Bno086Shtp::IsConfiguredReport(std::uint8_t report_id)
{
  return std::find(kConfiguredReports.begin(), kConfiguredReports.end(),
                   static_cast<ReportId>(report_id)) != kConfiguredReports.end();
}

std::size_t Bno086Shtp::SensorPayloadBytes(ReportId report_id)
{
  switch (report_id)
  {
    case ReportId::Accelerometer:
    case ReportId::GyroscopeCalibrated:
    case ReportId::LinearAcceleration:
    case ReportId::Gravity:
      return 6;
    case ReportId::RotationVector:
      return 10;
    default:
      break;
  }

  return 0;
}

std::optional<std::size_t> Bno086Shtp::SensorRecordBytes(std::uint8_t report_id)
{
  switch (report_id)
  {
    case static_cast<std::uint8_t>(ReportId::Accelerometer):
    case static_cast<std::uint8_t>(ReportId::GyroscopeCalibrated):
    case static_cast<std::uint8_t>(ReportId::LinearAcceleration):
    case static_cast<std::uint8_t>(ReportId::Gravity):
      return kSensorCommonHeaderBytes + 6;

    case static_cast<std::uint8_t>(ReportId::RotationVector):
      return kSensorCommonHeaderBytes + 10;

    default:
      break;
  }

  return std::nullopt;
}
} // namespace OASIS::IMU::BNO086
