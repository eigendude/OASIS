/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <cstdint>
#include <cstdlib>

namespace OASIS::IMU::BNO086
{
/*!\brief Coarse SHTP Poll result category used by warning policy */
enum class Bno086PollOutcome : std::uint8_t
{
  Timeout,
  TransportError,
  PacketHandled,
  SensorEvent,
};

/*!\brief Recent BNO086 runtime health used to suppress noisy warnings */
struct Bno086HealthSnapshot
{
  /*!
   * \brief True after at least one diagnostic rate window has completed
   *
   * Units: boolean
   */
  bool has_rate_window{false};

  /*!
   * \brief Reference accelerometer cadence for imu_gravity health
   *
   * Units: hertz, zero when unavailable
   */
  double expected_accel_rate_hz{0.0};

  /*!
   * \brief Latest imu_gravity publication rate
   *
   * Units: hertz over the latest diagnostic window
   */
  double imu_gravity_rate_hz{0.0};

  /*!
   * \brief Latest calibrated accelerometer report rate
   *
   * Units: hertz over the latest diagnostic window
   */
  double accel_rate_hz{0.0};

  /*!
   * \brief Latest orientation sample age used by imu_gravity
   *
   * Units: milliseconds
   */
  double orientation_age_ms{0.0};

  /*!
   * \brief Latest gyro sample age used by imu_gravity
   *
   * Units: milliseconds
   */
  double gyro_age_ms{0.0};

  /*!
   * \brief Configured maximum acceptable orientation age
   *
   * Units: milliseconds
   */
  double max_orientation_age_ms{0.0};

  /*!
   * \brief Configured maximum acceptable gyro age
   *
   * Units: milliseconds
   */
  double max_gyro_age_ms{0.0};

  /*!
   * \brief Accelerometer sequence gaps observed in the latest window
   *
   * Units: decoded report gaps
   */
  std::uint64_t accel_sequence_gap_delta{0};

  /*!
   * \brief Timestamp repairs observed in the latest window
   *
   * Units: repaired accelerometer reports
   */
  std::uint64_t timestamp_repair_delta{0};

  /*!
   * \brief imu_gravity skips caused by stale orientation in the latest window
   *
   * Units: skipped publications
   */
  std::uint64_t stale_orientation_skip_delta{0};

  /*!
   * \brief imu_gravity skips caused by stale gyro in the latest window
   *
   * Units: skipped publications
   */
  std::uint64_t stale_gyro_skip_delta{0};
};

/*!\brief Context for drain-exit warning decisions */
struct Bno086DrainExitWarningContext
{
  /*!
   * \brief Recent runtime health snapshot
   *
   * Units: structured diagnostics
   */
  Bno086HealthSnapshot health;

  /*!
   * \brief True when this drain is part of startup backlog handling
   *
   * Units: boolean
   */
  bool startup_backlog{false};

  /*!
   * \brief True when H_INTN remained asserted at drain exit
   *
   * Units: boolean
   */
  bool hintn_asserted_at_exit{false};

  /*!
   * \brief True when the drain exited through timeout handling
   *
   * Units: boolean
   */
  bool exited_timeout{false};

  /*!
   * \brief True when timeout occurred after packet/event progress
   *
   * Units: boolean
   */
  bool exited_timeout_after_progress{false};

  /*!
   * \brief True when the drain exited due to transport error
   *
   * Units: boolean
   */
  bool exited_transport_error{false};

  /*!
   * \brief True when the drain reached its wall-clock budget
   *
   * Units: boolean
   */
  bool exited_duration_budget{false};

  /*!
   * \brief True when the drain reached its packet safety cap
   *
   * Units: boolean
   */
  bool exited_max_packets{false};

  /*!
   * \brief Observed drain duration
   *
   * Units: milliseconds
   */
  double drain_duration_ms{0.0};

  /*!
   * \brief Configured drain duration budget
   *
   * Units: milliseconds
   */
  double max_drain_duration_ms{0.0};
};

/*!\brief Context for long Poll warning decisions */
struct Bno086PollWarningContext
{
  /*!
   * \brief Recent runtime health snapshot
   *
   * Units: structured diagnostics
   */
  Bno086HealthSnapshot health;

  /*!
   * \brief Poll outcome returned by the SHTP layer
   *
   * Units: enum code
   */
  Bno086PollOutcome outcome{Bno086PollOutcome::Timeout};

  /*!
   * \brief Observed Poll wall-clock duration
   *
   * Units: milliseconds
   */
  double poll_duration_ms{0.0};

  /*!
   * \brief Duration above which a Poll is considered long
   *
   * Units: milliseconds
   */
  double long_poll_threshold_ms{10.0};

  /*!
   * \brief Duration above which a Poll should always warn
   *
   * Units: milliseconds
   */
  double severe_poll_duration_ms{50.0};
};

/*!\brief Context for feature-rate warning decisions */
struct Bno086FeatureRateWarningContext
{
  /*!
   * \brief Rate requested by the Set Feature command
   *
   * Units: hertz, zero when unknown
   */
  double requested_rate_hz{0.0};

  /*!
   * \brief Rate accepted and reported by Get Feature Response
   *
   * Units: hertz, zero when unknown
   */
  double actual_rate_hz{0.0};
};

/*!\brief Context for timestamp-repair warning decisions */
struct Bno086TimestampRepairWarningContext
{
  /*!
   * \brief Absolute offset introduced by the latest timestamp repair
   *
   * Units: nanoseconds
   */
  std::int64_t repair_offset_ns{0};

  /*!
   * \brief Timestamp repairs observed in the latest diagnostic window
   *
   * Units: repaired reports
   */
  std::uint64_t repair_delta{0};

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
   * \brief Offset above which a repair should warn immediately
   *
   * Units: nanoseconds
   */
  std::int64_t warn_offset_ns{25'000'000};

  /*!
   * \brief Per-window repair count above which repairs should warn
   *
   * Units: repaired reports
   */
  std::uint64_t warn_delta_per_window{10};
};

/*!\brief Context for continuation-reset warning decisions */
struct Bno086ContinuationResetWarningContext
{
  /*!
   * \brief Recent runtime health snapshot
   *
   * Units: structured diagnostics
   */
  Bno086HealthSnapshot health;

  /*!
   * \brief Latest reset channel
   *
   * Units: SHTP channel number
   */
  std::uint8_t channel{0};

  /*!
   * \brief New resets since the previous continuation-reset log decision
   *
   * Units: resets
   */
  std::uint64_t reset_delta{0};

  /*!
   * \brief True after BNO086 communication has been established
   *
   * Units: boolean
   */
  bool communication_established{false};

  /*!
   * \brief True while the node is still draining startup backlog
   *
   * Units: boolean
   */
  bool startup_backlog{false};

  /*!
   * \brief Per-window reset count above which resets should warn
   *
   * Units: resets
   */
  std::uint64_t warn_delta_per_window{3};
};

inline bool IsBno086HealthyForWarningSuppression(const Bno086HealthSnapshot& health)
{
  if (!health.has_rate_window)
    return false;

  const double referenceAccelRateHz =
      health.expected_accel_rate_hz > 0.0 ? health.expected_accel_rate_hz : health.accel_rate_hz;
  const double minimumImuGravityRateHz = referenceAccelRateHz * 0.8;

  return health.imu_gravity_rate_hz >= minimumImuGravityRateHz &&
         health.orientation_age_ms <= health.max_orientation_age_ms &&
         health.gyro_age_ms <= health.max_gyro_age_ms && health.accel_sequence_gap_delta == 0 &&
         health.stale_orientation_skip_delta == 0 && health.stale_gyro_skip_delta == 0;
}

inline bool ShouldWarnBno086DrainExit(const Bno086DrainExitWarningContext& context)
{
  if (!context.hintn_asserted_at_exit)
    return false;

  if (context.exited_transport_error || context.exited_max_packets)
    return true;

  if (context.exited_timeout && !context.exited_timeout_after_progress)
    return true;

  if (!context.exited_duration_budget)
    return false;

  const bool severeDuration = context.max_drain_duration_ms > 0.0 &&
                              context.drain_duration_ms > (context.max_drain_duration_ms * 2.0);
  if (severeDuration)
    return true;

  if (context.startup_backlog && !context.health.has_rate_window)
    return false;

  return !IsBno086HealthyForWarningSuppression(context.health);
}

inline bool ShouldWarnBno086LongPoll(const Bno086PollWarningContext& context)
{
  if (context.poll_duration_ms <= context.long_poll_threshold_ms)
    return false;

  if (context.outcome == Bno086PollOutcome::Timeout ||
      context.outcome == Bno086PollOutcome::TransportError)
  {
    return true;
  }

  if (context.poll_duration_ms > context.severe_poll_duration_ms)
    return true;

  return !IsBno086HealthyForWarningSuppression(context.health);
}

inline bool ShouldWarnBno086FeatureRate(const Bno086FeatureRateWarningContext& context)
{
  if (context.requested_rate_hz <= 0.0 || context.actual_rate_hz <= 0.0)
    return false;

  return context.actual_rate_hz < (context.requested_rate_hz * 0.8);
}

inline bool ShouldWarnBno086TimestampRepair(const Bno086TimestampRepairWarningContext& context)
{
  if (std::llabs(context.repair_offset_ns) > context.warn_offset_ns)
    return true;

  if (context.repair_delta > context.warn_delta_per_window)
    return true;

  return context.stale_orientation_skip_delta > 0 || context.stale_gyro_skip_delta > 0;
}

inline bool ShouldWarnBno086ContinuationReset(const Bno086ContinuationResetWarningContext& context)
{
  if (context.reset_delta == 0)
    return false;

  if (context.channel == 0 && (!context.communication_established || context.startup_backlog))
    return false;

  if (context.reset_delta > context.warn_delta_per_window)
    return true;

  return !IsBno086HealthyForWarningSuppression(context.health);
}
} // namespace OASIS::IMU::BNO086
