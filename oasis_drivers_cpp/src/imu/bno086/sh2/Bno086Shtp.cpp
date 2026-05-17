/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/sh2/Bno086Shtp.hpp"

#include <algorithm>
#include <cmath>

namespace OASIS::IMU::BNO086
{
namespace
{
constexpr std::uint8_t kSensorFlushCompleted = 0xEF;
constexpr std::uint8_t kShtpProductIdResponse = 0xF8;
constexpr std::size_t kBaseTimestampHeaderBytes = 5;
constexpr std::size_t kSensorCommonHeaderBytes = 4;
constexpr std::size_t kFeatureResponseBytes = 17;
constexpr std::size_t kProductIdResponseBytes = 16;
constexpr std::size_t kGyroIntegratedRotationVectorPayloadBytes = 14;
constexpr std::size_t kMaxContinuationBytes = 4096;

constexpr std::array<ReportId, 5> kConfiguredReports = {
    ReportId::RotationVector,
    ReportId::GyroscopeCalibrated,
    ReportId::LinearAcceleration,
    ReportId::Accelerometer,
    ReportId::Gravity,
};

bool LooksLikeShtpHeaderPrefix(const std::vector<std::uint8_t>& payload)
{
  if (payload.size() < kShtpHeaderBytes)
    return false;

  return LooksLikeBno086ShtpHeaderPrefix(payload.data());
}
} // namespace

Bno086Shtp::Bno086Shtp(Bno086Transport& transport) : m_transport(transport)
{
}

bool Bno086Shtp::Configure(const Bno086ShtpConfig& config)
{
  m_config = config;
  m_config.report_rate_hz = std::max(m_config.report_rate_hz, 1.0);
  m_config.rotation_vector_rate_hz = std::max(m_config.rotation_vector_rate_hz, 1.0);
  m_config.gyro_rate_hz = std::max(m_config.gyro_rate_hz, 1.0);
  m_config.accelerometer_rate_hz = std::max(m_config.accelerometer_rate_hz, 1.0);
  m_config.linear_acceleration_rate_hz = std::max(m_config.linear_acceleration_rate_hz, 1.0);
  m_config.gravity_rate_hz = std::max(m_config.gravity_rate_hz, 1.0);

  m_featureConfigurations.clear();
  m_featureResponses.clear();

  for (const ReportId reportId : kConfiguredReports)
  {
    FeatureConfiguration featureConfiguration;
    featureConfiguration.report_id = reportId;
    featureConfiguration.enabled = RequestedEnabledForReport(reportId, m_config);
    if (featureConfiguration.enabled)
    {
      featureConfiguration.requested_interval_us = RequestedIntervalForReport(reportId, m_config);
      featureConfiguration.requested_batch_interval_us =
          RequestedBatchIntervalForReport(reportId, m_config);
    }
    else
    {
      featureConfiguration.requested_interval_us = 0;
      featureConfiguration.requested_batch_interval_us = 0;
    }
    m_featureConfigurations.emplace_back(featureConfiguration);
  }

  m_configRequested = true;
  m_startupStatus.set_feature_sent = false;
  m_lastConfigAttempt = {};

  return SendSetFeatureCommands();
}

Bno086Shtp::PollResult Bno086Shtp::Poll(int timeout_ms)
{
  PollResult result;

  if (!m_pendingEvents.empty())
  {
    result.event = m_pendingEvents.front();
    m_pendingEvents.erase(m_pendingEvents.begin());
    result.status = PollStatus::SensorEvent;
    result.dequeued_pending_event = true;
    return result;
  }

  Bno086ShtpPacket packet;
  if (!m_transport.ReadPacket(packet, timeout_ms))
    return result;

  result.read_physical_packet = true;

  if (!DecodePacket(packet, result.event))
  {
    result.status = PollStatus::TransportError;
    return result;
  }

  result.handled_control_packet = packet.channel < kShtpChannelReports;

  if (result.event.has_value())
  {
    result.status = PollStatus::SensorEvent;
    return result;
  }

  result.status = PollStatus::PacketHandled;
  return result;
}

Bno086Shtp::FeatureResponseDrainResult Bno086Shtp::DrainFeatureResponses(
    int timeout_ms, std::uint32_t max_physical_packets, int poll_timeout_ms)
{
  FeatureResponseDrainResult result;
  result.expected_responses = m_featureConfigurations.size();
  result.received_responses = m_featureResponses.size();
  result.complete = result.received_responses >= result.expected_responses;

  const auto startedAt = std::chrono::steady_clock::now();
  const auto timeout = std::chrono::milliseconds(std::max(timeout_ms, 0));
  const auto deadline = startedAt + timeout;

  while (!result.complete && result.physical_packets < max_physical_packets)
  {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline)
      break;

    const Bno086Shtp::PollResult pollResult = Poll(poll_timeout_ms);
    if (pollResult.read_physical_packet)
      ++result.physical_packets;

    if (pollResult.handled_control_packet)
      ++result.pre_report_packets;

    if (pollResult.status == PollStatus::SensorEvent && pollResult.event.has_value())
      ++result.sensor_events_seen;

    result.received_responses = m_featureResponses.size();
    result.complete = result.received_responses >= result.expected_responses;

    if (pollResult.status == PollStatus::TransportError)
      break;

    const bool madeProgress = pollResult.read_physical_packet || pollResult.dequeued_pending_event;
    if (!madeProgress && poll_timeout_ms <= 0)
      break;
  }

  result.elapsed_ms =
      static_cast<std::uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                     std::chrono::steady_clock::now() - startedAt)
                                     .count());
  result.received_responses = m_featureResponses.size();
  result.complete = result.received_responses >= result.expected_responses;
  return result;
}

const Bno086Shtp::StartupStatus& Bno086Shtp::GetStartupStatus() const
{
  return m_startupStatus;
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
    if (!ConfigureFeature(featureConfiguration))
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

bool Bno086Shtp::ConfigureFeature(const FeatureConfiguration& feature_configuration)
{
  std::vector<std::uint8_t> payload(17, 0);
  payload[0] = kShtpSetFeatureCommand;
  payload[1] = static_cast<std::uint8_t>(feature_configuration.report_id);

  WriteU32(payload, 5, feature_configuration.requested_interval_us);
  WriteU32(payload, 9, feature_configuration.requested_batch_interval_us);

  return m_transport.WritePacket(kShtpChannelControl, payload);
}

bool Bno086Shtp::RequestFeature(ReportId report_id)
{
  std::vector<std::uint8_t> payload(2, 0);
  payload[0] = kShtpGetFeatureRequest;
  payload[1] = static_cast<std::uint8_t>(report_id);

  return m_transport.WritePacket(kShtpChannelControl, payload);
}

bool Bno086Shtp::DecodePacket(const Bno086ShtpPacket& packet, std::optional<SensorEvent>& event)
{
  MarkCommunicationEstablished();

  Bno086ShtpPacket normalizedPacket = packet;
  if (IsSensorChannel(packet.channel))
  {
    std::vector<std::uint8_t> sensorPayload;
    if (packet.channel < m_continuationPayloads.size() && m_continuationActive[packet.channel])
    {
      auto& continuationPayload = m_continuationPayloads[packet.channel];
      sensorPayload.reserve(continuationPayload.size() + packet.payload.size());
      sensorPayload.insert(sensorPayload.end(), continuationPayload.begin(),
                           continuationPayload.end());
      sensorPayload.insert(sensorPayload.end(), packet.payload.begin(), packet.payload.end());
    }
    else
      sensorPayload = packet.payload;

    normalizedPacket.payload = sensorPayload;
    if (packet.channel < m_continuationPayloads.size())
    {
      m_continuationActive[packet.channel] = false;
      m_continuationPayloads[packet.channel].clear();
    }
  }

  if (packet.channel < m_continuationPayloads.size())
  {
    const bool parseCompleteControlContinuation = ShouldParseCompleteControlContinuation(packet);
    const ContinuationValidation validation = ValidateContinuationFragment(packet);
    if (!validation.valid)
    {
      if (validation.flush_buffer)
        ClearContinuationState(packet.channel);

      return true;
    }

    if (validation.keep_buffering && !IsSensorChannel(packet.channel))
    {
      auto& continuationPayload = m_continuationPayloads[packet.channel];
      m_continuationActive[packet.channel] = true;
      continuationPayload.insert(continuationPayload.end(), packet.payload.begin(),
                                 packet.payload.end());
      return true;
    }

    if (parseCompleteControlContinuation)
      ClearContinuationState(packet.channel);

    if (!IsSensorChannel(packet.channel) && m_continuationActive[packet.channel])
    {
      auto& continuationPayload = m_continuationPayloads[packet.channel];
      continuationPayload.insert(continuationPayload.end(), packet.payload.begin(),
                                 packet.payload.end());
      normalizedPacket.payload = continuationPayload;
      m_continuationActive[packet.channel] = false;
      continuationPayload.clear();
    }
  }

  if (normalizedPacket.channel < kShtpChannelReports)
  {
    if (normalizedPacket.channel == kShtpChannelControl)
      DecodeControlPayload(normalizedPacket.payload);

    MaybeSendDeferredConfiguration();
    return true;
  }

  if (IsSensorChannel(normalizedPacket.channel))
  {
    const SensorDecodeResult decoded =
        DecodeSensorPayload(normalizedPacket.payload, normalizedPacket.channel, event);

    if (packet.channel < m_continuationPayloads.size())
    {
      if (decoded.trailing_bytes > 0)
      {
        auto& continuationPayload = m_continuationPayloads[packet.channel];
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
      if (reportCode == kShtpProductIdResponse &&
          offset + kProductIdResponseBytes <= payload.size())
      {
        offset += kProductIdResponseBytes;
        continue;
      }

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
    std::optional<SensorEvent>& event)
{
  SensorDecodeResult result;
  event.reset();

  if (payload.empty())
    return result;

  if (channel == kShtpChannelGyroRotationVector)
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

    // SH-2 batches may contain a Base Timestamp control report followed by
    // several sensor reports. The timestamp value is encoded in 100 us ticks
    // and applies to each following sensor record until a later Base
    // Timestamp or Timestamp Rebase changes the local payload context.
    if (reportCode == kShtpReportBaseTimestamp)
    {
      if (offset + kBaseTimestampHeaderBytes > payload.size())
      {
        result.trailing_bytes = payload.size() - offset;
        break;
      }

      baseTimestampUs = ReadU32(payload, offset + 1) * 100U;
      offset += kBaseTimestampHeaderBytes;
      continue;
    }

    // Timestamp Rebase uses the same 100 us tick scale and increments the
    // current base timestamp context. If no base timestamp has appeared in
    // this decoded payload, the rebase has no context to update.
    if (reportCode == kShtpReportTimestampRebase)
    {
      if (offset + kBaseTimestampHeaderBytes > payload.size())
      {
        result.trailing_bytes = payload.size() - offset;
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
        break;
      }

      offset += kFlushCompletedBytes;
      continue;
    }

    const std::optional<std::size_t> recordBytes = SensorRecordBytes(reportCode);
    if (!recordBytes.has_value())
    {
      ++offset;
      continue;
    }

    if (offset + *recordBytes > payload.size())
    {
      result.trailing_bytes = payload.size() - offset;
      break;
    }

    SensorEvent decodedEvent;
    std::size_t bytesConsumed = 0;
    if (!DecodeSingleSensorReport(payload, offset, baseTimestampUs, decodedEvent, bytesConsumed))
    {
      offset += *recordBytes;
      continue;
    }

    decodedEvent.channel = channel;

    if (event.has_value())
      m_pendingEvents.emplace_back(decodedEvent);
    else
      event = decodedEvent;

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
  if (!IsReportEnabled(reportId))
    return false;

  const std::size_t payloadBytes = SensorPayloadBytes(reportId);
  const std::size_t recordBytes = kSensorCommonHeaderBytes + payloadBytes;
  if (report_offset + recordBytes > payload.size())
    return false;

  event = SensorEvent{};
  event.report_id = reportId;
  event.sequence = payload[report_offset + 1];
  event.status = payload[report_offset + 2];
  event.accuracy = static_cast<std::uint8_t>(event.status & 0x03U);
  event.report_offset = report_offset;
  event.bytes_consumed = recordBytes;

  // SH-2 combines accuracy in bits 1:0 with the upper six bits of the
  // 14-bit delay field in bits 7:2. Byte 3 contributes the lower eight bits.
  // This common four-byte report header is shared by the tracked vector and
  // rotation reports; only the value payload length differs by report type.
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

void Bno086Shtp::MarkCommunicationEstablished()
{
  m_startupStatus.communication_established = true;
}

bool Bno086Shtp::IsSensorChannel(std::uint8_t channel) const
{
  return channel == kShtpChannelReports || channel == kShtpChannelWakeReports ||
         channel == kShtpChannelGyroRotationVector;
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
      return validation;
    }

    if (packet.channel == kShtpChannelCommand && packet.payload.size() <= kShtpHeaderBytes)
    {
      validation.valid = false;
      validation.keep_buffering = false;
      validation.flush_buffer = true;
      return validation;
    }

    if (ShouldParseCompleteControlContinuation(packet))
    {
      validation.keep_buffering = false;
      return validation;
    }

    if (LooksLikeShtpHeaderPrefix(packet.payload))
    {
      validation.valid = false;
      validation.keep_buffering = false;
      validation.flush_buffer = true;
      return validation;
    }

    if ((bufferedBytes + packet.payload.size()) > kMaxContinuationBytes)
    {
      validation.valid = false;
      validation.keep_buffering = false;
      validation.flush_buffer = true;
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
    return validation;
  }

  if (LooksLikeShtpHeaderPrefix(packet.payload))
  {
    validation.valid = false;
    validation.flush_buffer = true;
    return validation;
  }

  if ((bufferedBytes + packet.payload.size()) > kMaxContinuationBytes)
  {
    validation.valid = false;
    validation.flush_buffer = true;
    return validation;
  }

  return validation;
}

bool Bno086Shtp::ShouldParseCompleteControlContinuation(const Bno086ShtpPacket& packet) const
{
  if (!packet.continuation || packet.channel != kShtpChannelControl)
    return false;

  if (packet.payload.size() < kFeatureResponseBytes)
    return false;

  // Some BNO086 firmware sets the SHTP continuation bit on standalone 17-byte
  // SH-2 Get Feature Responses. Treat those as complete control payloads so
  // feature verification can parse them.
  return packet.payload[0] == kShtpGetFeatureResponse;
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

std::uint32_t Bno086Shtp::RequestedIntervalForReport(ReportId report_id,
                                                     const Bno086ShtpConfig& config)
{
  switch (report_id)
  {
    case ReportId::RotationVector:
      return ToReportIntervalUs(config.rotation_vector_rate_hz);
    case ReportId::GyroscopeCalibrated:
      return ToReportIntervalUs(config.gyro_rate_hz);
    case ReportId::Accelerometer:
      return ToReportIntervalUs(config.accelerometer_rate_hz);
    case ReportId::LinearAcceleration:
      return ToReportIntervalUs(config.linear_acceleration_rate_hz);
    case ReportId::Gravity:
      return ToReportIntervalUs(config.gravity_rate_hz);
    default:
      break;
  }

  return ToReportIntervalUs(config.report_rate_hz);
}

std::uint32_t Bno086Shtp::RequestedBatchIntervalForReport(ReportId report_id,
                                                          const Bno086ShtpConfig& config)
{
  switch (report_id)
  {
    case ReportId::RotationVector:
      return config.rotation_vector_batch_interval_us;
    case ReportId::GyroscopeCalibrated:
      return config.gyro_batch_interval_us;
    case ReportId::Accelerometer:
      return config.accelerometer_batch_interval_us;
    case ReportId::LinearAcceleration:
      return config.linear_acceleration_batch_interval_us;
    case ReportId::Gravity:
      return config.gravity_batch_interval_us;
    default:
      break;
  }

  return 0;
}

bool Bno086Shtp::RequestedEnabledForReport(ReportId report_id, const Bno086ShtpConfig& config)
{
  switch (report_id)
  {
    case ReportId::LinearAcceleration:
      return config.enable_linear_acceleration_report;
    case ReportId::Gravity:
      return config.enable_gravity_report;
    default:
      break;
  }

  return true;
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

void Bno086Shtp::WriteU32(std::vector<std::uint8_t>& data, std::size_t offset, std::uint32_t value)
{
  if (offset + 3 >= data.size())
    return;

  data[offset] = static_cast<std::uint8_t>(value & 0xFF);
  data[offset + 1] = static_cast<std::uint8_t>((value >> 8) & 0xFF);
  data[offset + 2] = static_cast<std::uint8_t>((value >> 16) & 0xFF);
  data[offset + 3] = static_cast<std::uint8_t>((value >> 24) & 0xFF);
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

bool Bno086Shtp::IsReportEnabled(ReportId report_id) const
{
  const auto it = std::find_if(m_featureConfigurations.begin(), m_featureConfigurations.end(),
                               [report_id](const FeatureConfiguration& configuration)
                               { return configuration.report_id == report_id; });

  if (it == m_featureConfigurations.end())
    return true;

  return it->enabled;
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
