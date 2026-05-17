/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "imu/bno086/sh2/Bno086Reports.hpp"
#include "imu/bno086/shtp/Bno086Transport.hpp"

#include <array>
#include <chrono>
#include <optional>
#include <vector>

namespace OASIS::IMU::BNO086
{
struct Bno086ShtpConfig
{
  /*!
   * \brief Fallback BNO086 report generation rate
   *
   * Units: Hz, clamped to at least 1.0
   */
  double report_rate_hz{100.0};

  /*!
   * \brief Requested Rotation Vector generation rate
   *
   * Units: Hz, clamped to at least 1.0
   */
  double rotation_vector_rate_hz{50.0};

  /*!
   * \brief Requested Rotation Vector maximum batching delay
   *
   * Units: microseconds, zero disables batching
   */
  std::uint32_t rotation_vector_batch_interval_us{50'000};

  /*!
   * \brief Requested calibrated gyroscope generation rate
   *
   * Units: Hz, clamped to at least 1.0
   */
  double gyro_rate_hz{50.0};

  /*!
   * \brief Requested calibrated gyroscope maximum batching delay
   *
   * Units: microseconds, zero disables batching
   */
  std::uint32_t gyro_batch_interval_us{50'000};

  /*!
   * \brief Requested calibrated accelerometer generation rate
   *
   * Units: Hz, clamped to at least 1.0
   */
  double accelerometer_rate_hz{100.0};

  /*!
   * \brief Requested calibrated accelerometer maximum batching delay
   *
   * Units: microseconds, zero disables batching
   */
  std::uint32_t accelerometer_batch_interval_us{50'000};

  /*!
   * \brief Requested linear acceleration generation rate
   *
   * Units: Hz, clamped to at least 1.0
   */
  double linear_acceleration_rate_hz{50.0};

  /*!
   * \brief Requested linear acceleration maximum batching delay
   *
   * Units: microseconds, zero disables batching
   */
  std::uint32_t linear_acceleration_batch_interval_us{50'000};

  /*!
   * \brief Requested gravity vector generation rate
   *
   * Units: Hz, clamped to at least 1.0 when enabled
   */
  double gravity_rate_hz{25.0};

  /*!
   * \brief Requested gravity vector maximum batching delay
   *
   * Units: microseconds, zero disables batching
   */
  std::uint32_t gravity_batch_interval_us{100'000};

  /*!
   * \brief True when Linear Acceleration should be configured as enabled
   */
  bool enable_linear_acceleration_report{true};

  /*!
   * \brief True when the BNO086 Gravity report should be configured enabled
   */
  bool enable_gravity_report{true};
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

  struct PollResult
  {
    /*!
     * \brief High-level outcome of one polling attempt
     *
     * Units: enum code
     */
    PollStatus status{PollStatus::Timeout};

    /*!
     * \brief Decoded sensor event returned by this poll, when present
     *
     * Units: SH-2 report payload decoded into fixed-point fields
     */
    std::optional<SensorEvent> event;

    /*!
     * \brief True when this poll performed one physical SHTP/I2C packet read
     */
    bool read_physical_packet{false};

    /*!
     * \brief True when this poll returned an event already queued in memory
     */
    bool dequeued_pending_event{false};

    /*!
     * \brief True when this poll decoded a command/control-channel packet
     */
    bool handled_control_packet{false};

    /*!
     * \brief SHTP channel for the physical packet read by this poll
     *
     * Units: channel id, unset when no physical packet was read
     */
    std::optional<std::uint8_t> packet_channel;
  };

  struct StartupStatus
  {
    bool communication_established{false};
    bool set_feature_sent{false};
  };

  struct FeatureResponseDrainResult
  {
    /*!
     * \brief Number of Feature Responses available after the drain
     *
     * Units: response count
     */
    std::size_t received_responses{0};

    /*!
     * \brief Number of configured reports expected to answer
     *
     * Units: response count
     */
    std::size_t expected_responses{0};

    /*!
     * \brief Physical packets read while draining
     *
     * Units: packet count
     */
    std::uint32_t physical_packets{0};

    /*!
     * \brief SHTP packets with channel < reports observed while draining
     *
     * Units: packet count
     */
    std::uint32_t pre_report_packets{0};

    /*!
     * \brief Sensor events observed and ignored by the startup drain
     *
     * Units: event count
     */
    std::uint32_t sensor_events_seen{0};

    /*!
     * \brief Wall time spent in the drain loop
     *
     * Units: milliseconds
     */
    std::uint32_t elapsed_ms{0};

    /*!
     * \brief True when all expected Feature Responses were received
     */
    bool complete{false};
  };

  explicit Bno086Shtp(Bno086Transport& transport);

  bool Configure(const Bno086ShtpConfig& config);
  PollResult Poll(int timeout_ms);
  std::size_t PendingEventCount() const;
  FeatureResponseDrainResult DrainFeatureResponses(int timeout_ms,
                                                   std::uint32_t max_physical_packets,
                                                   int poll_timeout_ms);

  const StartupStatus& GetStartupStatus() const;
  const std::vector<FeatureConfiguration>& GetFeatureConfigurations() const;
  std::vector<FeatureResponse> TakeFeatureResponses();

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
  bool ConfigureFeature(const FeatureConfiguration& feature_configuration);
  bool RequestFeature(ReportId report_id);

  bool DecodePacket(const Bno086ShtpPacket& packet, std::optional<SensorEvent>& event);
  void DecodeControlPayload(const std::vector<std::uint8_t>& payload);
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
  bool ShouldParseCompleteControlContinuation(const Bno086ShtpPacket& packet) const;
  bool IsSensorChannel(std::uint8_t channel) const;
  void ClearContinuationState(std::uint8_t channel);

  static std::uint32_t ToReportIntervalUs(double rate_hz);
  static std::uint32_t RequestedIntervalForReport(ReportId report_id,
                                                  const Bno086ShtpConfig& config);
  static std::uint32_t RequestedBatchIntervalForReport(ReportId report_id,
                                                       const Bno086ShtpConfig& config);
  static bool RequestedEnabledForReport(ReportId report_id, const Bno086ShtpConfig& config);
  static std::uint32_t ReadU32(const std::vector<std::uint8_t>& data, std::size_t offset);
  static void WriteU32(std::vector<std::uint8_t>& data, std::size_t offset, std::uint32_t value);
  static std::int16_t ReadS16(const std::vector<std::uint8_t>& data, std::size_t offset);
  static bool IsTrackedReport(std::uint8_t report_id);
  static bool IsConfiguredReport(std::uint8_t report_id);
  bool IsReportEnabled(ReportId report_id) const;
  static std::size_t SensorPayloadBytes(ReportId report_id);
  static std::optional<std::size_t> SensorRecordBytes(std::uint8_t report_id);

  Bno086Transport& m_transport;
  Bno086ShtpConfig m_config{};

  StartupStatus m_startupStatus{};

  bool m_configRequested{false};
  std::vector<FeatureConfiguration> m_featureConfigurations;
  std::vector<FeatureResponse> m_featureResponses;

  std::chrono::steady_clock::time_point m_lastConfigAttempt{};
  std::vector<SensorEvent> m_pendingEvents;

  std::array<bool, 6> m_continuationActive{};
  std::array<std::vector<std::uint8_t>, 6> m_continuationPayloads{};
};
} // namespace OASIS::IMU::BNO086
