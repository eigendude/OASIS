/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "imu/bno086/Bno086Reports.hpp"
#include "imu/bno086/Bno086Transport.hpp"

#include <array>
#include <chrono>
#include <optional>
#include <vector>

namespace OASIS::IMU::BNO086
{
struct Bno086ShtpConfig
{
  // Requested internal BNO086 report generation rate in Hz
  // Converted to Set Feature report interval in microseconds
  double report_rate_hz{100.0};
};

class Bno086Shtp
{
public:
  enum class PollStatus
  {
    Timeout,
    PacketHandled,
    SensorEvent,
    TransportError,
  };

  struct StartupStatus
  {
    bool communication_established{false};
    bool set_feature_sent{false};
  };

  explicit Bno086Shtp(Bno086Transport& transport);

  bool Configure(const Bno086ShtpConfig& config);
  PollStatus Poll(std::optional<SensorEvent>& event, int timeout_ms);

  const StartupStatus& GetStartupStatus() const;

private:
  struct ContinuationValidation
  {
    bool valid{true};
    bool keep_buffering{false};
    bool flush_buffer{false};
  };

  struct SensorDecodeResult
  {
    std::size_t trailing_bytes{0};
  };

  bool SendSetFeatureCommands();
  bool ConfigureFeature(ReportId report_id, std::uint32_t interval_us);

  bool DecodePacket(const Bno086ShtpPacket& packet, std::optional<SensorEvent>& event);
  SensorDecodeResult DecodeSensorPayload(const std::vector<std::uint8_t>& payload,
                                         std::uint8_t channel,
                                         std::optional<SensorEvent>& event);
  bool DecodeSingleSensorReport(const std::vector<std::uint8_t>& payload,
                                std::size_t report_offset,
                                std::optional<std::uint32_t> base_timestamp_us,
                                SensorEvent& event,
                                std::size_t& bytes_consumed) const;

  void MaybeSendDeferredConfiguration();
  void MarkCommunicationEstablished();
  ContinuationValidation ValidateContinuationFragment(const Bno086ShtpPacket& packet) const;
  bool IsSensorChannel(std::uint8_t channel) const;
  void ClearContinuationState(std::uint8_t channel);

  static std::uint32_t ToReportIntervalUs(double rate_hz);
  static std::uint32_t ReadU32(const std::vector<std::uint8_t>& data, std::size_t offset);
  static std::int16_t ReadS16(const std::vector<std::uint8_t>& data, std::size_t offset);
  static bool IsTrackedReport(std::uint8_t report_id);
  static std::size_t SensorPayloadBytes(ReportId report_id);
  static std::optional<std::size_t> SensorRecordBytes(std::uint8_t report_id);

  Bno086Transport& m_transport;
  Bno086ShtpConfig m_config{};

  StartupStatus m_startupStatus{};

  bool m_configRequested{false};
  std::uint32_t m_reportIntervalUs{10'000};

  std::chrono::steady_clock::time_point m_lastConfigAttempt{};
  std::vector<SensorEvent> m_pendingEvents;

  std::array<bool, 6> m_continuationActive{};
  std::array<std::vector<std::uint8_t>, 6> m_continuationPayloads{};
};
} // namespace OASIS::IMU::BNO086
