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

class FullBrightness : public ILedBehavior
{
public:
  FullBrightness(LedVector leds);
  ~FullBrightness() override;

  // Implementation of ILedBehavior
  void Update(uint64_t runtimeMs, float progressRatio) override;
  void Shutdown() override;

private:
  // Construction parameters
  LedVector m_leds;
};

}
}
