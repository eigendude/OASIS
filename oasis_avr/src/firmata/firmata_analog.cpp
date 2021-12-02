/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  This file is derived from the FirmataExpress project and the AGPL-3. License
 *  Copyright (C) 2006-2008 Hans-Christoph Steiner. All rights reserved.
 *  Copyright (C) 2009-2017 Jeff Hoefs. All rights reserved.
 *  Copyright (C) 2018-2019 Alan Yorinks. All Rights Reserved.
 *
 *  SPDX-License-Identifier: Apache-2.0 AND AGPL-3
 *  See DOCS/LICENSING.md for more information.
 */

#include "firmata_analog.hpp"

#include <Boards.h>
#include <FirmataExpress.h>
#include <Scheduler.h>

using namespace OASIS;

namespace OASIS
{

// Threading constants
constexpr size_t ANALOG_STACK_SIZE = 64; // Default is 128

} // namespace OASIS

void FirmataAnalog::Setup(void (*loopFunc)())
{
  Scheduler.startLoop(loopFunc, ANALOG_STACK_SIZE);
}

void FirmataAnalog::Reset()
{
  // By default, do not report any analog inputs
  m_analogInputsToReport = 0;
}

void FirmataAnalog::Loop()
{
  m_reportTimer.SetTimeout(1000);

  for (uint8_t pin = 0; pin < TOTAL_PINS; ++pin)
  {
    if (IS_PIN_ANALOG(pin) && Firmata.getPinMode(pin) == PIN_MODE_ANALOG)
    {
      const uint8_t analogPin = PIN_TO_ANALOG(pin);

      if (m_analogInputsToReport & (1 << analogPin))
      {
        Firmata.sendAnalog(analogPin, analogRead(analogPin));
        yield();
      }
    }
  }

  delay(m_reportTimer.TimeLeft());
}

void FirmataAnalog::EnableAnalogInput(uint8_t analogPin, bool enable)
{
  if (enable)
  {
    m_analogInputsToReport |= (1 << analogPin);
  }
  else
  {
    m_analogInputsToReport &= ~(1 << analogPin);
  }
}
