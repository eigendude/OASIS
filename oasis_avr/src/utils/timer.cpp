/*
 *  Copyright (C) 2021-2024 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "timer.hpp"

#include <Arduino.h>

using namespace OASIS;

Timer::Timer()
{
  Reset();
}

bool Timer::IsExpired()
{
  // TODO: Handle wraparound
  return millis() - m_startMs >= m_intervalMs;
}

void Timer::SetTimeout(uint32_t intervalMs)
{
  m_intervalMs = intervalMs;
  m_startMs = millis();
}

void Timer::Reset()
{
  SetTimeout(0);
}

uint32_t Timer::TimeLeft() const
{
  return millis() + m_intervalMs - m_startMs;
}
