/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "PowerMeterCore.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <stdexcept>

namespace OASIS::PowerMeter
{
/** \brief Count raw samples that violate the active/apparent-power bound */
class ActivePowerInvariantMonitor
{
public:
  /** \brief Evaluate one raw sample with a nonnegative tolerance in watts */
  bool Evaluate(const Sample& raw_sample, double tolerance_watts)
  {
    if (tolerance_watts < 0.0)
      throw std::invalid_argument("active-power invariant tolerance must be nonnegative");
    const double apparent_power = raw_sample.voltage * raw_sample.current;
    if (std::abs(raw_sample.power) <= apparent_power + tolerance_watts)
      return false;
    ++m_violationCount;
    return true;
  }

  /** \brief Return the cumulative raw-sample violation count */
  std::uint64_t GetViolationCount() const { return m_violationCount; }

private:
  std::uint64_t m_violationCount{0};
};

/**
 * \brief Moving average over accepted ROS publication samples
 *
 * At 10 Hz, length three averages the current and previous two samples and
 * spans about 200 ms from oldest to newest. These are not necessarily three
 * contiguous 31.96875 ms hardware windows.
 */
class SampleMovingAverage
{
public:
  explicit SampleMovingAverage(std::size_t length) : m_length(length)
  {
    if (m_length == 0)
      throw std::invalid_argument("moving-average length must be positive");
  }

  /** \brief Add a raw sample and return the current component-wise mean */
  Sample Update(const Sample& sample)
  {
    m_samples.push_back(sample);
    if (m_samples.size() > m_length)
      m_samples.pop_front();

    Sample average = sample;
    average.voltage = 0.0;
    average.current = 0.0;
    average.power = 0.0;
    for (const Sample& entry : m_samples)
    {
      average.voltage += entry.voltage;
      average.current += entry.current;
      average.power += entry.power;
    }
    const double count = static_cast<double>(m_samples.size());
    average.voltage /= count;
    average.current /= count;
    average.power /= count;
    return average;
  }

  /** \brief Remove all history after failure, disconnect, or device reset */
  void Reset() { m_samples.clear(); }

  /** \brief Return the configured maximum history length */
  std::size_t GetLength() const { return m_length; }

private:
  std::size_t m_length;
  std::deque<Sample> m_samples;
};
} // namespace OASIS::PowerMeter
