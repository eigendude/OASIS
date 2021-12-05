/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "BusySignal.h"

#include "led/ILed.h"

using namespace OASIS;
using namespace LED;

namespace
{
constexpr unsigned int PERIOD_MS = 500;

constexpr unsigned int FORCE_TIMEOUT_MS = 5000;
} // namespace

BusySignal::BusySignal(LedVector leds) : m_leds(std::move(leds))
{
}

BusySignal::~BusySignal() = default;

void BusySignal::Update(uint64_t runtimeMs, float progressRatio)
{
  uint64_t elapsed = runtimeMs - m_lastEdgeMs;
  if (elapsed > PERIOD_MS / 2)
  {
    m_lastEdgeMs += PERIOD_MS / 2;
    m_active = !m_active;
  }

  for (LedPtr& led : m_leds)
  {
    if (m_active)
      led->Enable();
    else
      led->Disable();

    led->Update(runtimeMs < FORCE_TIMEOUT_MS);
  }
}

void BusySignal::Shutdown()
{
  for (LedPtr& led : m_leds)
  {
    led->Disable();
    led->Update(false);
  }
}
