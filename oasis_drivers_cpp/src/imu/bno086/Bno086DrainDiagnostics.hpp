/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <cstdint>

namespace OASIS::IMU::BNO086
{
/*!
 * \brief Interrupt and packet-drain diagnostics for the BNO086 driver
 *
 * All counters are cumulative since node startup. Timing fields are derived
 * from the interrupt steady-clock timestamp captured by the ROS node.
 */
struct InterruptDrainDiagnostics
{
  //! Number of H_INTN drain cycles started from observed interrupts
  std::uint64_t interrupt_received_count{0};

  //! Latest elapsed time from interrupt capture to first Poll call, in ms
  double latest_interrupt_to_first_poll_ms{0.0};

  //! Maximum elapsed time from interrupt capture to first Poll call, in ms
  double max_interrupt_to_first_poll_ms{0.0};

  //! Latest elapsed time from interrupt capture to last packet read, in ms
  double latest_interrupt_to_last_packet_ms{0.0};

  //! Maximum elapsed time from interrupt capture to last packet read, in ms
  double max_interrupt_to_last_packet_ms{0.0};

  //! Drain cycles whose total elapsed time exceeded 1 ms
  std::uint64_t drain_cycles_over_1ms{0};

  //! Drain cycles whose total elapsed time exceeded 5 ms
  std::uint64_t drain_cycles_over_5ms{0};

  //! Drain cycles whose total elapsed time exceeded 10 ms
  std::uint64_t drain_cycles_over_10ms{0};

  //! Drain cycles whose total elapsed time exceeded 20 ms
  std::uint64_t drain_cycles_over_20ms{0};

  //! Interrupt responses whose first Poll call exceeded the 10 ms guidance
  std::uint64_t interrupt_response_over_10ms{0};

  //! Number of interrupt drain loops entered
  std::uint64_t drain_cycles_started{0};

  //! Number of interrupt drain loops completed
  std::uint64_t drain_cycles_completed{0};

  //! Packets read during the latest completed drain cycle
  std::uint64_t packets_per_drain_latest{0};

  //! Maximum packets read during any completed drain cycle
  std::uint64_t packets_per_drain_max{0};

  //! Total packets read across completed drain cycles
  std::uint64_t packets_per_drain_total{0};

  //! Drain exits caused by Poll returning Timeout after retry handling
  std::uint64_t drain_exited_timeout{0};

  //! Timeout exits after at least one packet or sensor event was handled
  std::uint64_t drain_exited_timeout_after_progress{0};

  //! Timeout exits before any packet or sensor event was handled
  std::uint64_t drain_exited_timeout_no_progress{0};

  //! Timeout exits where H_INTN was deasserted when the timeout was handled
  std::uint64_t drain_exited_timeout_hintn_deasserted{0};

  //! Timeout exits where H_INTN was still asserted when retries were exhausted
  std::uint64_t drain_exited_timeout_hintn_asserted{0};

  //! Drain exits where H_INTN was deasserted when the exit state was sampled
  std::uint64_t drain_exited_int_deasserted{0};

  //! Drain exits caused by reaching the per-interrupt packet budget
  std::uint64_t drain_exited_max_packets{0};

  //! Drain exits caused by reaching the per-interrupt duration budget
  std::uint64_t drain_exited_duration_budget{0};

  //! Drain exits caused by a transport error from Poll
  std::uint64_t drain_exited_transport_error{0};

  //! Drain cycles that completed without decoded sensor events
  std::uint64_t drain_exited_no_events{0};

  //! Sensor events decoded during the latest completed drain cycle
  std::uint64_t sensor_events_per_drain_latest{0};

  //! Maximum sensor events decoded during any completed drain cycle
  std::uint64_t sensor_events_per_drain_max{0};

  //! Configured maximum drain duration, in ms
  double max_drain_duration_ms_config{0.0};

  //! Latest completed drain duration, in ms
  double latest_drain_duration_ms{0.0};

  //! Maximum completed drain duration observed, in ms
  double max_drain_duration_ms_observed{0.0};

  //! Packets read per ms during the latest completed drain cycle
  double packets_per_ms_latest{0.0};

  //! Sensor events decoded per ms during the latest completed drain cycle
  double sensor_events_per_ms_latest{0.0};

  //! Drain cycles considered part of startup backlog characterization
  std::uint64_t startup_backlog_drains{0};

  //! Whether H_INTN was asserted when the latest drain exit state was sampled
  bool latest_hintn_asserted_at_exit{false};

  //! Drain exits where H_INTN was still asserted when exit state was sampled
  std::uint64_t count_hintn_still_asserted_at_exit{0};

  //! Whether H_INTN was asserted during the latest packet-read timeout
  bool latest_timeout_hintn_asserted{false};

  //! Packet-read timeout retries attempted while H_INTN was asserted
  std::uint64_t timeout_retries_while_hintn_asserted{0};

  //! Timeout retries used during the latest completed drain cycle
  std::uint64_t latest_timeout_retries_while_hintn_asserted{0};

  //! Consecutive completed drain cycles that exited with H_INTN asserted
  std::uint64_t consecutive_hintn_asserted_drain_exits{0};

  //! Maximum consecutive drain exits observed with H_INTN asserted
  std::uint64_t max_consecutive_hintn_asserted_drain_exits{0};

  //! Count of asserted-exit streaks beyond the configured recovery threshold
  std::uint64_t stuck_interrupt_recovery_candidate_count{0};
};

/*! \brief Result of handling a packet-read timeout during one drain cycle */
struct TimeoutRetryDecision
{
  //! True when the drain loop should sleep briefly and call Poll again
  bool retry{false};

  //! True when the drain loop should exit through timeout accounting
  bool exit_timeout{false};
};

/*!
 * \brief Update drain duration and throughput diagnostics
 *
 * \param diagnostics Drain diagnostics updated in place
 * \param drain_duration_ms Completed drain duration, in ms
 * \param packets_read Packets read during the drain
 * \param sensor_events_decoded Sensor events decoded during the drain
 * \param max_drain_duration_ms Configured drain duration budget, in ms
 */
inline void RecordDrainDuration(InterruptDrainDiagnostics& diagnostics,
                                double drain_duration_ms,
                                std::uint64_t packets_read,
                                std::uint64_t sensor_events_decoded,
                                double max_drain_duration_ms)
{
  diagnostics.max_drain_duration_ms_config = max_drain_duration_ms;
  diagnostics.latest_drain_duration_ms = drain_duration_ms;
  if (drain_duration_ms > diagnostics.max_drain_duration_ms_observed)
    diagnostics.max_drain_duration_ms_observed = drain_duration_ms;

  if (drain_duration_ms > 0.0)
  {
    diagnostics.packets_per_ms_latest = static_cast<double>(packets_read) / drain_duration_ms;
    diagnostics.sensor_events_per_ms_latest =
        static_cast<double>(sensor_events_decoded) / drain_duration_ms;
  }
  else
  {
    diagnostics.packets_per_ms_latest = 0.0;
    diagnostics.sensor_events_per_ms_latest = 0.0;
  }
}

/*!
 * \brief Update timeout diagnostics and decide whether one drain should retry
 *
 * \param diagnostics Drain diagnostics updated in place
 * \param hintn_asserted True when H_INTN is active at timeout handling time
 * \param made_progress True after the drain handled any packets or events
 * \param timeout_retries_used Retries already consumed in this drain cycle
 * \param max_timeout_retries Retry budget for asserted-H_INTN timeouts
 *
 * \return A retry/exit decision for the caller's drain loop
 */
inline TimeoutRetryDecision HandleDrainTimeout(InterruptDrainDiagnostics& diagnostics,
                                               bool hintn_asserted,
                                               bool made_progress,
                                               int timeout_retries_used,
                                               int max_timeout_retries)
{
  diagnostics.latest_timeout_hintn_asserted = hintn_asserted;

  if (!made_progress && hintn_asserted && timeout_retries_used < max_timeout_retries)
  {
    ++diagnostics.timeout_retries_while_hintn_asserted;
    return TimeoutRetryDecision{true, false};
  }

  ++diagnostics.drain_exited_timeout;
  if (made_progress)
    ++diagnostics.drain_exited_timeout_after_progress;
  else
    ++diagnostics.drain_exited_timeout_no_progress;

  if (hintn_asserted)
    ++diagnostics.drain_exited_timeout_hintn_asserted;
  else
    ++diagnostics.drain_exited_timeout_hintn_deasserted;

  return TimeoutRetryDecision{false, true};
}

/*!
 * \brief Handle a no-progress packet-read timeout during one drain cycle
 *
 * \param diagnostics Drain diagnostics updated in place
 * \param hintn_asserted True when H_INTN is active at timeout handling time
 * \param timeout_retries_used Retries already consumed in this drain cycle
 * \param max_timeout_retries Retry budget for asserted-H_INTN timeouts
 *
 * \return A retry/exit decision for the caller's drain loop
 */
inline TimeoutRetryDecision HandleTimeoutWhileDraining(InterruptDrainDiagnostics& diagnostics,
                                                       bool hintn_asserted,
                                                       int timeout_retries_used,
                                                       int max_timeout_retries)
{
  return HandleDrainTimeout(diagnostics, hintn_asserted, false, timeout_retries_used,
                            max_timeout_retries);
}

/*!
 * \brief Update counters that describe the sampled H_INTN state at drain exit
 *
 * \param diagnostics Drain diagnostics updated in place
 * \param hintn_asserted_at_exit True when H_INTN is active at drain exit
 * \param recovery_threshold Consecutive asserted exits before recovery count
 */
inline void RecordDrainExitHintnState(InterruptDrainDiagnostics& diagnostics,
                                      bool hintn_asserted_at_exit,
                                      std::uint64_t recovery_threshold)
{
  diagnostics.latest_hintn_asserted_at_exit = hintn_asserted_at_exit;

  if (!hintn_asserted_at_exit)
  {
    diagnostics.consecutive_hintn_asserted_drain_exits = 0;
    return;
  }

  ++diagnostics.count_hintn_still_asserted_at_exit;
  ++diagnostics.consecutive_hintn_asserted_drain_exits;
  if (diagnostics.consecutive_hintn_asserted_drain_exits >
      diagnostics.max_consecutive_hintn_asserted_drain_exits)
  {
    diagnostics.max_consecutive_hintn_asserted_drain_exits =
        diagnostics.consecutive_hintn_asserted_drain_exits;
  }

  if (diagnostics.consecutive_hintn_asserted_drain_exits > recovery_threshold)
    ++diagnostics.stuck_interrupt_recovery_candidate_count;
}
} // namespace OASIS::IMU::BNO086
