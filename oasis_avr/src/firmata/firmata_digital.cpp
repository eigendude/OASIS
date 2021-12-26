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

#include "firmata_digital.hpp"

#include <FirmataExpress.h>
#include <Scheduler.h>

using namespace OASIS;

namespace OASIS
{

// Threading constants
constexpr size_t DIGITAL_STACK_SIZE = 96; // Default is 128

} // namespace OASIS

void FirmataDigital::Setup(void (*loopFunc)())
{
  Scheduler.startLoop(loopFunc, DIGITAL_STACK_SIZE);
}

void FirmataDigital::Reset()
{
  for (uint8_t i = 0; i < TOTAL_PORTS; ++i)
  {
    // By default, reporting off
    m_reportPINs[i] = 0;
    m_portConfigInputs[i] = 0; // Until activated
    m_previousPINs[i] = 0;
  }

  // Send digital inputs to set the initial state on the host computer, since
  // once in the loop(), this firmware will only send on change
  //
  // TODO: This can never execute, since no pins default to digital input
  // but it will be needed when/if we support EEPROM stored config
  /*
  for (uint8_t i = 0; i < TOTAL_PORTS; ++i)
  {
    OutputPort(i, readPort(i, m_portConfigInputs[i]), true);
  }
  */
}

void FirmataDigital::Loop()
{
  bool forceSend = false;

  if (m_reportTimer.IsExpired())
  {
    forceSend = true;
    m_reportTimer.SetTimeout(1000);
  }

  // Read as fast as possible, check for changes and output them to the FTDI
  // buffer
  CheckDigitalInputs(forceSend);

  yield();
}

void FirmataDigital::SetDigitalPinMode(uint8_t digitalPin, int pinMode)
{
  const uint8_t digitalPort = digitalPin / 8;
  const uint8_t portBit = digitalPin & 7;

  if (pinMode == PIN_MODE_INPUT || pinMode == PIN_MODE_PULLUP)
  {
    m_portConfigInputs[digitalPort] |= (1 << portBit);
  }
  else
  {
    m_portConfigInputs[digitalPort] &= ~(1 << portBit);
  }
}

void FirmataDigital::EnableDigitalInput(uint8_t digitalPort, bool enable)
{
  m_reportPINs[digitalPort] = enable ? 1 : 0;
}

void FirmataDigital::SendPort(uint8_t digitalPort)
{
  OutputPort(digitalPort, readPort(digitalPort, m_portConfigInputs[digitalPort]), true);
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
