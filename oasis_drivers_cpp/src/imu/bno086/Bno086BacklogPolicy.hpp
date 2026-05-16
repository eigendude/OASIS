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
/*!\brief Thresholds used to classify BNO086 packet backlog */
struct Bno086BacklogDetectionConfig
{
  /*!
   * \brief Mean decoded reports per SHTP packet that indicates packet buildup
   *
   * Units: decoded sensor reports per packet
   */
  double events_per_packet_threshold{4.0};

  /*!
   * \brief Mean full raw-packet read size that indicates packet buildup
   *
   * Units: bytes per full-packet I2C transaction
   */
  double full_packet_bytes_threshold{64.0};

  /*!
   * \brief Minimum imu_gravity rate relative to decoded accelerometer rate
   *
   * Units: unitless ratio in range [0, 1]
   */
  double imu_gravity_rate_ratio_threshold{0.8};

  /*!
   * \brief Drain duration multiplier that indicates severe backlog
   *
   * Units: unitless multiplier of configured drain duration
   */
  double severe_drain_duration_multiplier{2.0};
};

/*!\brief Latest diagnostic-window sample used for backlog detection */
struct Bno086BacklogSample
{
  /*!
   * \brief Mean decoded reports per SHTP packet in the latest window
   *
   * Units: decoded sensor reports per packet
   */
  double events_per_packet_mean{0.0};

  /*!
   * \brief Mean full raw-packet read size in the latest window
   *
   * Units: bytes per full-packet I2C transaction
   */
  double full_packet_read_bytes_mean{0.0};

  /*!
   * \brief Latest imu_gravity publication rate
   *
   * Units: hertz over the latest diagnostic window
   */
  double imu_gravity_rate_hz{0.0};

  /*!
   * \brief Latest decoded calibrated accelerometer report rate
   *
   * Units: hertz over the latest diagnostic window
   */
  double accel_decoded_rate_hz{0.0};

  /*!
   * \brief Accelerometer sequence gaps observed in the latest window
   *
   * Units: decoded report gaps
   */
  std::uint64_t accel_sequence_gap_delta{0};

  /*!
   * \brief imu_gravity stale-orientation skips in the latest window
   *
   * Units: skipped publications
   */
  std::uint64_t stale_orientation_skip_delta{0};

  /*!
   * \brief imu_gravity stale-gyro skips in the latest window
   *
   * Units: skipped publications
   */
  std::uint64_t stale_gyro_skip_delta{0};

  /*!
   * \brief Latest completed drain duration
   *
   * Units: milliseconds
   */
  double latest_drain_duration_ms{0.0};

  /*!
   * \brief Configured drain duration budget
   *
   * Units: milliseconds
   */
  double max_drain_duration_ms{0.0};
};

/*!\brief Configuration for adaptive BNO086 optional-report rate limiting */
struct Bno086AdaptiveRateLimitConfig
{
  /*!
   * \brief True when adaptive rate limiting is allowed
   *
   * Units: boolean
   */
  bool enabled{true};

  /*!
   * \brief Consecutive backlog windows before entering adaptive mode
   *
   * Units: diagnostic windows
   */
  std::uint64_t backlog_windows_before_rate_limit{2};

  /*!
   * \brief Consecutive healthy windows before restoring configured rates
   *
   * Units: diagnostic windows
   */
  std::uint64_t recovery_windows_before_restore{6};

  /*!
   * \brief Startup interval during which non-severe backlog does not adapt
   *
   * Units: seconds
   */
  double startup_grace_sec{10.0};
};

/*!\brief Mutable adaptive rate-limit state */
struct Bno086AdaptiveRateLimitState
{
  /*!
   * \brief True when optional-report fallback rates are active
   *
   * Units: boolean
   */
  bool active{false};

  /*!
   * \brief Backlog classification for the latest diagnostic window
   *
   * Units: boolean
   */
  bool latest_backlog_detected{false};

  /*!
   * \brief Total number of diagnostic windows classified as backlog
   *
   * Units: diagnostic windows
   */
  std::uint64_t backlog_detected_count{0};

  /*!
   * \brief Current consecutive backlog-window streak
   *
   * Units: diagnostic windows
   */
  std::uint64_t consecutive_backlog_windows{0};

  /*!
   * \brief Maximum consecutive backlog-window streak observed
   *
   * Units: diagnostic windows
   */
  std::uint64_t max_consecutive_backlog_windows{0};

  /*!
   * \brief Current consecutive non-backlog-window streak
   *
   * Units: diagnostic windows
   */
  std::uint64_t consecutive_recovery_windows{0};

  /*!
   * \brief Number of transitions into adaptive mode
   *
   * Units: transitions
   */
  std::uint64_t adaptive_rate_limit_entries{0};

  /*!
   * \brief Number of transitions out of adaptive mode
   *
   * Units: transitions
   */
  std::uint64_t adaptive_rate_limit_exits{0};
};

/*!\brief Result of one adaptive rate-limit state update */
struct Bno086AdaptiveRateLimitDecision
{
  /*!
   * \brief True when the caller should apply fallback report intervals
   *
   * Units: boolean
   */
  bool enter_adaptive_mode{false};

  /*!
   * \brief True when the caller should restore configured report intervals
   *
   * Units: boolean
   */
  bool exit_adaptive_mode{false};
};

inline bool IsBno086BacklogDetected(const Bno086BacklogSample& sample,
                                    const Bno086BacklogDetectionConfig& config)
{
  const bool packetPressure =
      sample.events_per_packet_mean > config.events_per_packet_threshold ||
      sample.full_packet_read_bytes_mean > config.full_packet_bytes_threshold;
  const bool lowImuGravityRate =
      sample.accel_decoded_rate_hz > 0.0 &&
      sample.imu_gravity_rate_hz <
          (sample.accel_decoded_rate_hz * config.imu_gravity_rate_ratio_threshold);
  const bool staleSkips =
      sample.stale_orientation_skip_delta > 0 || sample.stale_gyro_skip_delta > 0;
  const bool severeDrain =
      sample.max_drain_duration_ms > 0.0 &&
      sample.latest_drain_duration_ms >
          (sample.max_drain_duration_ms * config.severe_drain_duration_multiplier);
  const bool healthPressure =
      lowImuGravityRate || sample.accel_sequence_gap_delta > 0 || staleSkips || severeDrain;

  return packetPressure && healthPressure;
}

inline bool IsBno086SevereBacklog(const Bno086BacklogSample& sample,
                                  const Bno086BacklogDetectionConfig& config)
{
  const bool highPacketPressure =
      sample.events_per_packet_mean > (config.events_per_packet_threshold * 2.0) ||
      sample.full_packet_read_bytes_mean > (config.full_packet_bytes_threshold * 2.0);
  const bool sequenceOrStalePressure = sample.accel_sequence_gap_delta > 0 ||
                                       sample.stale_orientation_skip_delta > 0 ||
                                       sample.stale_gyro_skip_delta > 0;

  return highPacketPressure && sequenceOrStalePressure;
}

inline Bno086AdaptiveRateLimitDecision UpdateBno086AdaptiveRateLimit(
    Bno086AdaptiveRateLimitState& state,
    const Bno086BacklogSample& sample,
    const Bno086BacklogDetectionConfig& detection_config,
    const Bno086AdaptiveRateLimitConfig& adaptive_config,
    double seconds_since_start)
{
  Bno086AdaptiveRateLimitDecision decision;
  const bool backlogDetected = IsBno086BacklogDetected(sample, detection_config);
  state.latest_backlog_detected = backlogDetected;

  if (backlogDetected)
  {
    ++state.backlog_detected_count;
    ++state.consecutive_backlog_windows;
    state.consecutive_recovery_windows = 0;
    if (state.consecutive_backlog_windows > state.max_consecutive_backlog_windows)
      state.max_consecutive_backlog_windows = state.consecutive_backlog_windows;
  }
  else
  {
    state.consecutive_backlog_windows = 0;
    if (state.active)
      ++state.consecutive_recovery_windows;
    else
      state.consecutive_recovery_windows = 0;
  }

  if (!adaptive_config.enabled)
    return decision;

  const bool withinStartupGrace = seconds_since_start < adaptive_config.startup_grace_sec;
  const bool severeBacklog = IsBno086SevereBacklog(sample, detection_config);
  const bool startupAllowsEntry = !withinStartupGrace || severeBacklog;

  if (!state.active && backlogDetected && startupAllowsEntry &&
      state.consecutive_backlog_windows >= adaptive_config.backlog_windows_before_rate_limit)
  {
    state.active = true;
    state.consecutive_recovery_windows = 0;
    ++state.adaptive_rate_limit_entries;
    decision.enter_adaptive_mode = true;
    return decision;
  }

  if (state.active && !backlogDetected &&
      state.consecutive_recovery_windows >= adaptive_config.recovery_windows_before_restore)
  {
    state.active = false;
    state.consecutive_backlog_windows = 0;
    ++state.adaptive_rate_limit_exits;
    decision.exit_adaptive_mode = true;
  }

  return decision;
}
} // namespace OASIS::IMU::BNO086
