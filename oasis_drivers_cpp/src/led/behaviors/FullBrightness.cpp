/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "FullBrightness.h"

#include "led/ILed.h"

using namespace OASIS;
using namespace LED;

namespace
{
constexpr unsigned int FORCE_TIMEOUT_MS = 5000;
}

FullBrightness::FullBrightness(LedVector leds) : m_leds(std::move(leds))
{
}

FullBrightness::~FullBrightness() = default;

void FullBrightness::Update(uint64_t runtimeMs, float progressRatio)
{
  for (LedPtr& led : m_leds)
  {
    led->Enable();
    led->Update(runtimeMs < FORCE_TIMEOUT_MS);
  }
}

void FullBrightness::Shutdown()
{
  for (LedPtr& led : m_leds)
  {
    led->Disable();
    led->Update(false);
  }
}
