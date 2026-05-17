/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/core/Bno086DrainPolicy.hpp"

namespace OASIS::IMU::BNO086
{
bool Bno086DrainMadeProgress(const Bno086DrainCounters& counters)
{
  return counters.physical_packets_this_drain > 0 || counters.sensor_events_this_drain > 0 ||
         counters.control_packets_this_drain > 0;
}

Bno086DrainDecision Bno086DrainBeforePoll(const Bno086DrainLimits& limits,
                                          const Bno086DrainCounters& counters,
                                          bool hintn_asserted)
{
  Bno086DrainDecision decision;
  decision.hintn_asserted = hintn_asserted;

  if (counters.poll_iterations >= limits.max_poll_iterations_per_interrupt)
    decision.action = Bno086DrainAction::PollIterationCap;

  return decision;
}

Bno086DrainDecision Bno086DrainAfterPoll(const Bno086Shtp::PollResult& result,
                                         const Bno086DrainLimits& limits,
                                         Bno086DrainCounters& counters,
                                         bool hintn_asserted)
{
  Bno086DrainDecision decision;
  decision.hintn_asserted = hintn_asserted;

  ++counters.poll_iterations;

  if (result.status == Bno086Shtp::PollStatus::Timeout ||
      result.status == Bno086Shtp::PollStatus::AllZeroHeader)
  {
    ++counters.consecutive_no_progress_polls;
    if (result.status == Bno086Shtp::PollStatus::AllZeroHeader)
    {
      ++counters.all_zero_polls_this_drain;
      ++counters.consecutive_all_zero_polls;
    }
    else
    {
      counters.consecutive_all_zero_polls = 0;
    }

    if (!hintn_asserted)
      decision.action = Bno086DrainAction::Complete;
    else if (result.status == Bno086Shtp::PollStatus::AllZeroHeader &&
             counters.consecutive_all_zero_polls >= limits.max_all_zero_polls_per_interrupt)
      decision.action = Bno086DrainAction::AllZeroBudget;
    else if (counters.consecutive_no_progress_polls >= limits.max_no_progress_polls_per_interrupt)
      decision.action = Bno086DrainAction::NoProgressBudget;
    else
      decision.action = Bno086DrainAction::Continue;

    return decision;
  }

  if (result.read_physical_packet)
    ++counters.physical_packets_this_drain;

  if (result.event.has_value())
    ++counters.sensor_events_this_drain;

  if (result.dequeued_pending_event)
    ++counters.pending_events_this_drain;

  if (result.handled_control_packet)
    ++counters.control_packets_this_drain;

  if (result.read_physical_packet || result.event.has_value() || result.dequeued_pending_event ||
      result.handled_control_packet)
  {
    counters.consecutive_no_progress_polls = 0;
    counters.consecutive_all_zero_polls = 0;
  }

  if (result.status == Bno086Shtp::PollStatus::TransportError)
  {
    decision.action = Bno086DrainAction::TransportError;
    return decision;
  }

  if (counters.physical_packets_this_drain >= limits.max_physical_packets_per_interrupt)
  {
    decision.action = Bno086DrainAction::PhysicalPacketCap;
    return decision;
  }

  if (result.dequeued_pending_event &&
      counters.pending_events_this_drain >= limits.max_pending_events_flush_per_drain)
  {
    decision.action = Bno086DrainAction::PendingEventFlushBudget;
    return decision;
  }

  if (!result.dequeued_pending_event &&
      counters.sensor_events_this_drain >= limits.max_sensor_events_per_drain)
  {
    decision.action = Bno086DrainAction::SensorEventBudget;
    return decision;
  }

  return decision;
}
} // namespace OASIS::IMU::BNO086
