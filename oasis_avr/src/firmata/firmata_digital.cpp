/*
 *  Copyright (C) 2021-2025 Garrett Brown
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

#include "firmata_digital.hpp"

#include <Arduino.h>
#include <FirmataExpress.h>
#include <Scheduler.h>

using namespace OASIS;

namespace
{

// Timing parameters
constexpr unsigned int FORCE_SEND_INTERVAL_MS = 1000;

} // namespace

void FirmataDigital::Loop()
{
  bool forceSend = false;

  if (m_reportTimer.IsExpired())
  {
    forceSend = true;
    m_reportTimer.SetTimeout(FORCE_SEND_INTERVAL_MS);
  }

  // Read as fast as possible, check for changes and output them to the FTDI
  // buffer
  CheckDigitalInputs(forceSend);
}

void FirmataDigital::SetDigitalPinMode(uint8_t digitalPin, int mode)
{
  const uint8_t digitalPort = digitalPin / 8;
  const uint8_t portBit = digitalPin & 7;

  if (mode == PIN_MODE_INPUT || mode == PIN_MODE_PULLUP)
  {
    m_portConfigInputs[digitalPort] |= (1 << portBit);
  }
  else
  {
    m_portConfigInputs[digitalPort] &= ~(1 << portBit);
  }

  if (mode == PIN_MODE_INPUT)
    pinMode(digitalPin, INPUT);
  else if (mode == PIN_MODE_PULLUP)
    pinMode(digitalPin, INPUT_PULLUP);
  else if (mode == PIN_MODE_OUTPUT || mode == PIN_MODE_PWM)
    pinMode(digitalPin, OUTPUT);

  Firmata.setPinMode(digitalPin, mode);
  Firmata.setPinState(digitalPin, 0);
}

void FirmataDigital::DisableDigitalReporting(uint8_t digitalPin)
{
  const uint8_t digitalPort = digitalPin / 8;
  const uint8_t portBit = digitalPin & 7;

  m_portConfigInputs[digitalPort] &= ~(1 << portBit);
}

void FirmataDigital::EnableDigitalInput(uint8_t digitalPort, bool enable)
{
  if (digitalPort < TOTAL_PORTS)
  {
    m_reportPINs[digitalPort] = enable ? 1 : 0;

    // Send port value immediately. This is helpful when connected via ethernet,
    // WiFi or Bluetooth so pin states can be known upon reconnecting.
    if (enable)
      SendPort(digitalPort);
  }
}

void FirmataDigital::SendPort(uint8_t digitalPort)
{
  OutputPort(digitalPort, readPort(digitalPort, m_portConfigInputs[digitalPort]), true);
}

void FirmataDigital::DigitalWrite(uint8_t digitalPort, int portValue)
{
  uint8_t mask = 1;
  uint8_t pinWriteMask = 0;

  // Create a mask of the pins on this port that are writable.
  uint8_t lastPin = digitalPort * 8 + 8;

  if (lastPin > TOTAL_PINS)
    lastPin = TOTAL_PINS;

  for (uint8_t pin = digitalPort * 8; pin < lastPin; ++pin)
  {
    // Do not disturb non-digital pins (eg, Rx & Tx)
    if (IS_PIN_DIGITAL(pin))
    {
      // Do not touch pins in PWM, ANALOG, SERVO or other modes
      if (Firmata.getPinMode(pin) == PIN_MODE_OUTPUT || Firmata.getPinMode(pin) == PIN_MODE_INPUT)
      {
        uint8_t pinValue = (static_cast<uint8_t>(portValue) & mask) ? 1 : 0;

        if (Firmata.getPinMode(pin) == PIN_MODE_OUTPUT)
        {
          pinWriteMask |= mask;
        }
        else if (Firmata.getPinMode(pin) == PIN_MODE_INPUT && pinValue == 1 &&
                 Firmata.getPinState(pin) != 1)
        {
          // Only handle INPUT here for backwards compatibility
          pinMode(pin, INPUT_PULLUP);
        }

        Firmata.setPinState(pin, pinValue);
      }
    }

    mask = mask << 1;
  }

  writePort(digitalPort, static_cast<uint8_t>(portValue), pinWriteMask);
}

void FirmataDigital::PWMWrite(uint8_t digitalPin, int analogValue)
{
  if (digitalPin < TOTAL_PINS)
  {
    if (IS_PIN_PWM(digitalPin))
      analogWrite(PIN_TO_PWM(digitalPin), analogValue);

    Firmata.setPinState(digitalPin, analogValue);
  }
}

void FirmataDigital::OutputPort(uint8_t digitalPort, uint8_t portValue, bool forceSend)
{
  // Pins not configured as INPUT are cleared to zeros
  portValue = portValue & m_portConfigInputs[digitalPort];

  // Only send if the value is different than previously sent
  if (forceSend || m_previousPINs[digitalPort] != portValue)
  {
    Firmata.sendDigitalPort(digitalPort, portValue);
    m_previousPINs[digitalPort] = portValue;
  }
}

void FirmataDigital::CheckDigitalInputs(bool forceSend)
{
  // Using non-looping code allows constants to be given to readPort(). The
  // compiler will apply substantial optimizations if the inputs to readPort()
  // are compile-time constants.
  if (TOTAL_PORTS > 0 && m_reportPINs[0])
    OutputPort(0, readPort(0, m_portConfigInputs[0]), forceSend);
  if (TOTAL_PORTS > 1 && m_reportPINs[1])
    OutputPort(1, readPort(1, m_portConfigInputs[1]), forceSend);
  if (TOTAL_PORTS > 2 && m_reportPINs[2])
    OutputPort(2, readPort(2, m_portConfigInputs[2]), forceSend);
  if (TOTAL_PORTS > 3 && m_reportPINs[3])
    OutputPort(3, readPort(3, m_portConfigInputs[3]), forceSend);
  if (TOTAL_PORTS > 4 && m_reportPINs[4])
    OutputPort(4, readPort(4, m_portConfigInputs[4]), forceSend);
  if (TOTAL_PORTS > 5 && m_reportPINs[5])
    OutputPort(5, readPort(5, m_portConfigInputs[5]), forceSend);
  if (TOTAL_PORTS > 6 && m_reportPINs[6])
    OutputPort(6, readPort(6, m_portConfigInputs[6]), forceSend);
  if (TOTAL_PORTS > 7 && m_reportPINs[7])
    OutputPort(7, readPort(7, m_portConfigInputs[7]), forceSend);
  if (TOTAL_PORTS > 8 && m_reportPINs[8])
    OutputPort(8, readPort(8, m_portConfigInputs[8]), forceSend);
  if (TOTAL_PORTS > 9 && m_reportPINs[9])
    OutputPort(9, readPort(9, m_portConfigInputs[9]), forceSend);
  if (TOTAL_PORTS > 10 && m_reportPINs[10])
    OutputPort(10, readPort(10, m_portConfigInputs[10]), forceSend);
  if (TOTAL_PORTS > 11 && m_reportPINs[11])
    OutputPort(11, readPort(11, m_portConfigInputs[11]), forceSend);
  if (TOTAL_PORTS > 12 && m_reportPINs[12])
    OutputPort(12, readPort(12, m_portConfigInputs[12]), forceSend);
  if (TOTAL_PORTS > 13 && m_reportPINs[13])
    OutputPort(13, readPort(13, m_portConfigInputs[13]), forceSend);
  if (TOTAL_PORTS > 14 && m_reportPINs[14])
    OutputPort(14, readPort(14, m_portConfigInputs[14]), forceSend);
  if (TOTAL_PORTS > 15 && m_reportPINs[15])
    OutputPort(15, readPort(15, m_portConfigInputs[15]), forceSend);
}
