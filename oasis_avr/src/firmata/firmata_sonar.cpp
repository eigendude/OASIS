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

#include "firmata_sonar.hpp"

#include "firmata_callbacks.hpp"
#include "utils/delete.hpp"

#include <Arduino.h>
#include <FirmataExpress.h>
#include <Scheduler.h>
#include <Ultrasonic.h>

using namespace OASIS;

namespace OASIS
{

// Threading constants
constexpr size_t SONAR_STACK_SIZE = 64; // Default is 128

} // namespace OASIS

void FirmataSonar::Setup(void (*loopFunc)())
{
  Scheduler.startLoop(loopFunc, SONAR_STACK_SIZE);
}

void FirmataSonar::Reset()
{
  // Stop pinging
  m_numActiveSonars = 0;

  for (int i = 0; i < MAX_SONARS; i++)
  {
    m_sonarPinNumbers[i] = PIN_MODE_IGNORE;
    Ultrasonic*& sonar = m_sonars[i];
    if (sonar != nullptr)
    {
      delete sonar;
      sonar = nullptr;
    }
  }

  m_numActiveSonars = 0;
}

void FirmataSonar::Loop()
{
  if (m_numActiveSonars)
  {
    unsigned int distance = m_sonars[m_nextSonar]->read();
    m_currentSonar = m_nextSonar;
    if (m_nextSonar++ >= m_numActiveSonars - 1)
      m_nextSonar = 0;

    m_sonarLSB = distance & 0x7f;
    m_sonarMSB = distance >> 7 & 0x7f;

    Firmata.write(START_SYSEX);
    Firmata.write(SONAR_DATA);
    Firmata.write(m_sonarPinNumbers[m_currentSonar]);
    Firmata.write(m_sonarLSB);
    Firmata.write(m_sonarMSB);
    Firmata.write(END_SYSEX);
  }

  // TODO
  delay(1000);
}

void FirmataSonar::AddSonar(uint8_t sonarTriggerPin, uint8_t sonarEchoPin, unsigned long timeout)
{
  m_sonarTriggerPin = sonarTriggerPin;
  m_sonarEchoPin = sonarEchoPin;

  m_sonarPinNumbers[m_numActiveSonars] = sonarTriggerPin;

  FirmataCallbacks::SetPinModeCallback(sonarTriggerPin, PIN_MODE_SONAR);
  FirmataCallbacks::SetPinModeCallback(sonarEchoPin, PIN_MODE_SONAR);
  m_sonars[m_numActiveSonars] = new Ultrasonic(sonarTriggerPin, sonarEchoPin, timeout);

  m_numActiveSonars++;
}
