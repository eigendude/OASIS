/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "imu/bno086/sh2/Bno086Shtp.hpp"

#include <cstdint>

namespace OASIS::IMU::BNO086
{
/*!\brief Drain-loop action selected after applying progress and cap rules */
enum class Bno086DrainAction
{
  Continue,
  Complete,
  WarnNoProgressTimeout,
  TransportError,
  PhysicalPacketCap,
  PollIterationCap,
  NoProgressBudget,
  SensorEventBudget,
  DrainDurationBudget,
};

struct Bno086DrainLimits
{
  /*!
   * \brief Maximum physical SHTP/I2C packet reads in one interrupt drain
   *
   * Units: packet reads, expected range [1, 8192]
   */
  std::uint32_t max_physical_packets_per_interrupt{1024};

  /*!
   * \brief Maximum Poll() calls in one interrupt drain
   *
   * Units: poll iterations, expected range [1, 16384]
   */
  std::uint32_t max_poll_iterations_per_interrupt{4096};

  /*!
   * \brief Maximum consecutive no-progress polls while H_INTN is asserted
   *
   * Units: poll iterations, expected range [1, 1024]
   */
  std::uint32_t max_no_progress_polls_per_interrupt{64};

  /*!
   * \brief Maximum decoded SensorEvent objects handled in one drain
   *
   * Units: decoded events, expected range [1, 4096]
   */
  std::uint32_t max_sensor_events_per_drain{512};
};

struct Bno086DrainCounters
{
  /*!
   * \brief Number of Poll() calls made during the drain
   *
   * Units: iterations, incremented before classifying each PollResult
   */
  std::uint32_t poll_iterations{0};

  /*!
   * \brief Number of real SHTP/I2C packet reads during the drain
   *
   * Units: physical packet reads from the transport
   */
  std::uint32_t physical_packets_this_drain{0};

  /*!
   * \brief Number of decoded SensorEvent objects handled during the drain
   *
   * Units: decoded events, including physical and pending events
   */
  std::uint32_t sensor_events_this_drain{0};

  /*!
   * \brief Number of SensorEvent objects returned from the pending queue
   *
   * Units: decoded events already buffered from earlier packet decoding
   */
  std::uint32_t pending_events_this_drain{0};

  /*!
   * \brief Number of command/control-channel packets handled during the drain
   *
   * Units: physical packet reads on SHTP command/control channels
   */
  std::uint32_t control_packets_this_drain{0};

  /*!
   * \brief Consecutive polls that returned no packet or pending event
   *
   * Units: poll iterations, reset when any poll makes progress
   */
  std::uint32_t consecutive_no_progress_polls{0};
};

struct Bno086DrainDecision
{
  /*!
   * \brief Action the caller should take after this drain policy step
   *
   * Units: enum code
   */
  Bno086DrainAction action{Bno086DrainAction::Continue};

  /*!
   * \brief True when H_INTN remained asserted at the decision point
   */
  bool hintn_asserted{false};
};

bool Bno086DrainMadeProgress(const Bno086DrainCounters& counters);
Bno086DrainDecision Bno086DrainBeforePoll(const Bno086DrainLimits& limits,
                                          const Bno086DrainCounters& counters,
                                          bool hintn_asserted);
Bno086DrainDecision Bno086DrainAfterPoll(const Bno086Shtp::PollResult& result,
                                         const Bno086DrainLimits& limits,
                                         Bno086DrainCounters& counters,
                                         bool hintn_asserted);
} // namespace OASIS::IMU::BNO086
