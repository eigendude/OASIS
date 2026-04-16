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
constexpr std::uint8_t kChannelControl = 2;
constexpr std::uint8_t kChannelReports = 3;
constexpr std::uint8_t kChannelWakeReports = 4;
constexpr std::uint8_t kChannelGyroRotationVector = 5;

constexpr std::uint8_t kSensorFlushCompleted = 0xEF;
constexpr std::size_t kBaseTimestampHeaderBytes = 5;
constexpr std::size_t kSensorCommonHeaderBytes = 4;
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

  const std::uint16_t lengthField =
      static_cast<std::uint16_t>(payload[0] | (static_cast<std::uint16_t>(payload[1]) << 8));
  const std::size_t packetLength = static_cast<std::size_t>(lengthField & 0x7FFFU);
  return packetLength >= kShtpHeaderBytes && packetLength <= 1024;
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

  m_configRequested = true;
  m_startupStatus.set_feature_sent = false;
  m_lastConfigAttempt = {};

  return SendSetFeatureCommands();
}

Bno086Shtp::PollStatus Bno086Shtp::Poll(std::optional<SensorEvent>& event, int timeout_ms)
{
  event.reset();

  if (!m_pendingEvents.empty())
  {
    event = m_pendingEvents.front();
    m_pendingEvents.erase(m_pendingEvents.begin());
    return PollStatus::SensorEvent;
  }

  Bno086ShtpPacket packet;
  if (!m_transport.ReadPacket(packet, timeout_ms))
    return PollStatus::Timeout;

  if (!DecodePacket(packet, event))
    return PollStatus::TransportError;

  if (event.has_value())
    return PollStatus::SensorEvent;

  return PollStatus::PacketHandled;
}

const Bno086Shtp::StartupStatus& Bno086Shtp::GetStartupStatus() const
{
  return m_startupStatus;
}

bool Bno086Shtp::SendSetFeatureCommands()
{
  if (!m_configRequested)
    return true;

  bool allOk = true;
  for (const ReportId reportId : kConfiguredReports)
  {
    if (!ConfigureFeature(reportId, m_reportIntervalUs))
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

  if (normalizedPacket.channel < kChannelReports)
  {
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

Bno086Shtp::SensorDecodeResult Bno086Shtp::DecodeSensorPayload(
    const std::vector<std::uint8_t>& payload,
    std::uint8_t channel,
    std::optional<SensorEvent>& event)
{
  SensorDecodeResult result;
  event.reset();

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
      return validation;
    }

    if (packet.channel == kChannelCommand && packet.payload.size() <= kShtpHeaderBytes)
    {
      validation.valid = false;
      validation.keep_buffering = false;
      validation.flush_buffer = true;
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
