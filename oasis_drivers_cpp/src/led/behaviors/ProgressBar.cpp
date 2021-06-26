/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "ProgressBar.h"
#include "led/ILed.h"

using namespace OASIS;
using namespace LED;

namespace
{
  constexpr unsigned int FORCE_TIMEOUT_MS = 5000;
}

ProgressBar::ProgressBar(LedVector leds) :
  m_leds(std::move(leds))
{
}

ProgressBar::~ProgressBar() = default;

void ProgressBar::Update(uint64_t runtimeMs, float progressRatio)
{
  for (unsigned int i = 0; i < m_leds.size(); i++)
  {
    LedPtr &led = m_leds[i];

    if (1.0f * i / m_leds.size() < progressRatio)
      led->Enable();
    else
      led->Disable();

    led->Update(runtimeMs < FORCE_TIMEOUT_MS);
  }
}

void ProgressBar::Shutdown()
{
  for (LedPtr &led : m_leds)
  {
    led->Disable();
    led->Update(false);
  }
}
