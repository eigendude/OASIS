/*
 *  Copyright (C) 2021-2023 Garrett Brown
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

#include "firmata_analog.hpp"

#include <Boards.h>
#include <FirmataExpress.h>
#include <Scheduler.h>

using namespace OASIS;

void FirmataAnalog::Sample()
{
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
}

void FirmataAnalog::SetAnalogMode(uint8_t pin)
{
  if (IS_PIN_ANALOG(pin))
  {
    Firmata.setPinMode(pin, PIN_MODE_ANALOG);
    EnableAnalogInput(PIN_TO_ANALOG(pin), true);
  }
}

void FirmataAnalog::EnableAnalogInput(uint8_t analogPin, bool enable)
{
  if (analogPin < TOTAL_ANALOG_PINS)
  {
    if (enable)
    {
      m_analogInputsToReport |= (1 << analogPin);

      // Send pin value immediately. This is helpful when connected via
      // ethernet, WiFi or Bluetooth so pin states can be known upon
      // reconnecting.
      Firmata.sendAnalog(analogPin, analogRead(analogPin));
    }
    else
    {
      m_analogInputsToReport &= ~(1 << analogPin);
    }
  }
}
