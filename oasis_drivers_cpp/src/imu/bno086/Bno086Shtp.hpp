/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "imu/bno086/Bno086Reports.hpp"
#include "imu/bno086/Bno086TimestampReconstructor.hpp"
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

/*!\brief Reason a buffered SHTP continuation payload was discarded */
enum class ContinuationResetReason : std::uint8_t
{
  None,
  EmptyPayload,
  CommandHeaderOnly,
  ShtpHeaderPrefix,
  MaxBytesExceeded,
};

/*!\brief Decoded sequence diagnostics for one SH-2 report stream */
struct ReportSequenceDiagnostics
{
  /*!
   * \brief True after at least one sequence has been decoded
   *
   * Units: boolean
   */
  bool has_sequence{false};

  /*!
   * \brief Last decoded sequence for this report stream
   *
   * Units: uint8 counter ticks, wraps at 255
   */
  std::uint8_t last_sequence{0};

  /*!
   * \brief Count of decoded samples where sequence advanced by more than one
   *
   * Units: samples
   */
  std::uint64_t gap_count{0};

  /*!
   * \brief Largest observed wrapped sequence delta
   *
   * Units: counter ticks in range [0, 255]
   */
  std::uint8_t gap_max{0};

  /*!
   * \brief Count of decoded samples with the same sequence as the prior sample
   *
   * Units: samples
   */
  std::uint64_t duplicate_sequence_count{0};
};

/*!\brief Runtime diagnostics for SHTP packet delivery and decode */
struct ShtpDiagnostics
{
  /*!
   * \brief Count of packets read from the transport
   *
   * Units: packets
   */
  std::uint64_t packets_read{0};

  /*!
   * \brief Count of packets read on any sensor report channel
   *
   * Units: packets
   */
  std::uint64_t sensor_packets_read{0};

  /*!
   * \brief Count of packets read on the control channel
   *
   * Units: packets
   */
  std::uint64_t control_packets_read{0};

  /*!
   * \brief Count of packets read on the executable channel
   *
   * Units: packets
   */
  std::uint64_t executable_packets_read{0};

  /*!
   * \brief Count of packets read on the wake reports channel
   *
   * Units: packets
   */
  std::uint64_t wake_packets_read{0};

  /*!
   * \brief Count of decoded SH-2 sensor reports
   *
   * Units: events
   */
  std::uint64_t sensor_events_decoded{0};

  /*!
   * \brief Count of unrecognized report bytes in sensor payloads
   *
   * Units: report bytes
   */
  std::uint64_t unknown_sensor_reports{0};

  /*!
   * \brief Count of malformed sensor payload records
   *
   * Units: records
   */
  std::uint64_t malformed_sensor_payloads{0};

  /*!
   * \brief Count of sensor decode errors
   *
   * Units: errors
   */
  std::uint64_t decode_errors{0};

  /*!
   * \brief Count of continuation buffers started
   *
   * Units: buffers
   */
  std::uint64_t continuation_packets_started{0};

  /*!
   * \brief Count of continuation buffers completed
   *
   * Units: buffers
   */
  std::uint64_t continuation_packets_completed{0};

  /*!
   * \brief Count of continuation buffers discarded or reset
   *
   * Units: buffers
   */
  std::uint64_t continuation_packets_reset{0};

  /*!
   * \brief Size of the latest packet payload read
   *
   * Units: bytes
   */
  std::uint32_t latest_packet_payload_bytes{0};

  /*!
   * \brief Largest packet payload read
   *
   * Units: bytes
   */
  std::uint32_t max_packet_payload_bytes{0};

  /*!
   * \brief Number of decoded events in the latest sensor packet
   *
   * Units: events
   */
  std::uint32_t latest_events_per_packet{0};

  /*!
   * \brief Largest number of decoded events observed in one packet
   *
   * Units: events
   */
  std::uint32_t max_events_per_packet{0};

  /*!
   * \brief Latest continuation reset channel
   *
   * Units: SHTP channel id
   */
  std::uint8_t latest_continuation_reset_channel{0};

  /*!
   * \brief Buffered bytes present when the latest continuation reset occurred
   *
   * Units: bytes
   */
  std::uint32_t latest_continuation_reset_accumulated_bytes{0};

  /*!
   * \brief Incoming payload bytes when the latest continuation reset occurred
   *
   * Units: bytes
   */
  std::uint32_t latest_continuation_reset_incoming_bytes{0};

  /*!
   * \brief Reason for the latest continuation reset
   *
   * Units: enum code
   */
  ContinuationResetReason latest_continuation_reset_reason{ContinuationResetReason::None};

  /*!
   * \brief Decoded report counts indexed by SH-2 report id
   *
   * Units: events
   */
  std::array<std::uint64_t, 256> decoded_report_counts{};

  /*!
   * \brief Sequence diagnostics indexed by SH-2 report id
   *
   * Units: report-specific counters
   */
  std::array<ReportSequenceDiagnostics, 256> report_sequence{};
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
  PollStatus Poll(std::vector<TimestampedSensorEvent>& events,
                  int timeout_ms,
                  std::int64_t packet_host_stamp_ns);

  const StartupStatus& GetStartupStatus() const;
  const ShtpDiagnostics& GetDiagnostics() const;
  const TimestampReconstructionDiagnostics& GetTimestampReconstructionDiagnostics() const;
  const std::vector<FeatureConfiguration>& GetFeatureConfigurations() const;
  std::vector<FeatureResponse> TakeFeatureResponses();

private:
  struct ContinuationValidation
  {
    bool valid{true};
    bool keep_buffering{false};
    bool flush_buffer{false};
    ContinuationResetReason reset_reason{ContinuationResetReason::None};
  };

  struct SensorDecodeResult
  {
    std::size_t trailing_bytes{0};
    bool malformed_payload{false};
    bool decode_error{false};
  };

  bool SendSetFeatureCommands();
  bool ConfigureFeature(ReportId report_id, std::uint32_t interval_us);
  bool RequestFeature(ReportId report_id);

  bool DecodePacket(const Bno086ShtpPacket& packet,
                    std::vector<TimestampedSensorEvent>& events,
                    std::int64_t packet_host_stamp_ns);
  void DecodeControlPayload(const std::vector<std::uint8_t>& payload);
  SensorDecodeResult DecodeSensorPayload(const std::vector<std::uint8_t>& payload,
                                         std::uint8_t channel,
                                         std::vector<TimestampedSensorEvent>& events,
                                         std::int64_t packet_host_stamp_ns,
                                         bool payload_can_continue);
  bool DecodeSingleSensorReport(const std::vector<std::uint8_t>& payload,
                                std::size_t report_offset,
                                std::optional<std::uint32_t> base_timestamp_us,
                                SensorEvent& event,
                                std::size_t& bytes_consumed) const;

  void RecordPacketRead(const Bno086ShtpPacket& packet);
  void RecordDecodedSensorEvent(const SensorEvent& event);
  void RecordContinuationReset(std::uint8_t channel,
                               std::size_t accumulated_bytes,
                               std::size_t incoming_bytes,
                               ContinuationResetReason reason);
  void MaybeSendDeferredConfiguration();
  void MarkCommunicationEstablished();
  ContinuationValidation ValidateContinuationFragment(const Bno086ShtpPacket& packet) const;
  bool IsSensorChannel(std::uint8_t channel) const;
  void ClearContinuationState(std::uint8_t channel);

  static std::uint32_t ToReportIntervalUs(double rate_hz);
  static std::uint32_t RequestedIntervalForReport(ReportId report_id,
                                                  std::uint32_t configured_interval_us);
  static std::uint32_t ReadU32(const std::vector<std::uint8_t>& data, std::size_t offset);
  static std::int16_t ReadS16(const std::vector<std::uint8_t>& data, std::size_t offset);
  static bool IsTrackedReport(std::uint8_t report_id);
  static bool IsConfiguredReport(std::uint8_t report_id);
  static std::size_t SensorPayloadBytes(ReportId report_id);
  static std::optional<std::size_t> SensorRecordBytes(std::uint8_t report_id);

  Bno086Transport& m_transport;
  Bno086ShtpConfig m_config{};

  StartupStatus m_startupStatus{};

  bool m_configRequested{false};
  std::uint32_t m_reportIntervalUs{10'000};
  std::vector<FeatureConfiguration> m_featureConfigurations;
  std::vector<FeatureResponse> m_featureResponses;
  Bno086TimestampReconstructor m_timestampReconstructor;
  ShtpDiagnostics m_diagnostics{};

  std::chrono::steady_clock::time_point m_lastConfigAttempt{};

  std::array<bool, 6> m_continuationActive{};
  std::array<std::vector<std::uint8_t>, 6> m_continuationPayloads{};
};
} // namespace OASIS::IMU::BNO086
