/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */
#pragma once

#include "led/ILedBehavior.h"
#include "led/LedTypes.h"

namespace OASIS
{
namespace LED
{

class BusySignal : public ILedBehavior
{
public:
  BusySignal(LedVector leds);
  ~BusySignal() override;

  // Implementation of ILedBehavior
  void Update(uint64_t runtimeMs, float progressRatio) override;
  void Shutdown() override;

private:
  // Construction parameters
  LedVector m_leds;

  // State parameters
  uint64_t m_lastEdgeMs = 0;
  bool m_active = false;
};

}
}
