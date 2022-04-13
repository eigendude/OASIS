/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  This file is derived from FirmataExpress under the AGPL 3.0 License
 *  Copyright (C) 2006-2008 Hans-Christoph Steiner. All rights reserved.
 *  Copyright (C) 2009-2017 Jeff Hoefs. All rights reserved.
 *  Copyright (C) 2018-2019 Alan Yorinks. All Rights Reserved.
 *
 *  SPDX-License-Identifier: Apache-2.0 AND AGPL-3.0
 *  See DOCS/LICENSING.md for more information.
 */
#pragma once

#include "utils/timer.hpp"

#include <stdint.h>

namespace OASIS
{

class FirmataAnalog
{
public:
  // Lifecycle functions
  void Setup(void (*loopFunc)());
  void Reset();
  void Loop();

  // Analog pin functions
  void SetAnalogMode(uint8_t pin);
  void EnableAnalogInput(uint8_t analogPin, bool enable);

private:
  // Analog pin state
  uint16_t m_analogInputsToReport = 0; // Bitwise array to store pin reporting

  // Timing parameters
  Timer m_reportTimer;
};

} // namespace OASIS
