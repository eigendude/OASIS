/*
 *  Copyright (C) 2021-2025 Garrett Brown
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

#include <Arduino.h>
#include <FirmataExpress.h>
#include <Ultrasonic.h>

using namespace OASIS;

void FirmataSonar::Sample()
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
}

void FirmataSonar::SetSonarMode(uint8_t digitalPin)
{
  Firmata.setPinMode(digitalPin, PIN_MODE_SONAR);
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
