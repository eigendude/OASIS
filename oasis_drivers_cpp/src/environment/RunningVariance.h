/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <cstddef>
#include <deque>

namespace OASIS::Environment
{
class RunningVariance
{
public:
  explicit RunningVariance(std::size_t capacity) : m_capacity(capacity) {}

  void Add(double value)
  {
    if (m_capacity == 0)
      return;
    if (m_values.size() == m_capacity)
    {
      const double removed = m_values.front();
      m_values.pop_front();
      if (m_values.empty())
      {
        m_mean = 0.0;
        m_m2 = 0.0;
      }
      else
      {
        const double old_count = static_cast<double>(m_values.size() + 1);
        const double new_mean = (old_count * m_mean - removed) / (old_count - 1.0);
        m_m2 -= (removed - m_mean) * (removed - new_mean);
        m_mean = new_mean;
      }
    }

    const double new_count = static_cast<double>(m_values.size() + 1);
    const double delta = value - m_mean;
    m_mean += delta / new_count;
    m_m2 += delta * (value - m_mean);
    m_values.push_back(value);
  }

  [[nodiscard]] double Variance() const
  {
    if (m_values.size() < 2)
      return 0.0;
    const double variance = m_m2 / static_cast<double>(m_values.size() - 1);
    return variance > 0.0 ? variance : 0.0;
  }

  [[nodiscard]] std::size_t Size() const { return m_values.size(); }

  void Reset()
  {
    m_values.clear();
    m_mean = 0.0;
    m_m2 = 0.0;
  }

private:
  std::size_t m_capacity;
  std::deque<double> m_values;
  double m_mean{0.0};
  double m_m2{0.0};
};
} // namespace OASIS::Environment
