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
#pragma once

#include "firmata_subsystem.hpp"
#include "utils/timer.hpp"

#include <stdint.h>

#include <Boards.h>

class Ultrasonic;

namespace OASIS
{

class FirmataSonar : public FirmataSubsystem
{
public:
  // Implementation of FirmataSubsystem
  void Sample() override;

  // Sonar functions
  void SetSonarMode(uint8_t digitalPin);
  uint8_t GetActiveSonarCount() const { return m_numActiveSonars; }
  void AddSonar(uint8_t sonarTriggerPin, uint8_t sonarEchoPin, unsigned long timeout);

private:
  // Sonar state
  int m_numActiveSonars = 0; // Number of sonars attached
  uint8_t m_sonarPinNumbers[MAX_SONARS]{};
  int m_nextSonar = 0; // Index into m_sonars[] for next device
  Ultrasonic* m_sonars[MAX_SONARS]{}; // Array to hold up to 6 instances of sonar devices
  uint8_t m_sonarTriggerPin = 0;
  uint8_t m_sonarEchoPin = 0;
  uint8_t m_currentSonar = 0; // Keeps track of which sensor is active.
  // Milliseconds between sensor pings (29ms is about the min to avoid
  // cross- sensor echo)
  //static const uint8_t m_pingIntervalMs = 33; // TODO: Unused?
  uint8_t m_sonarMSB = 0;
  uint8_t m_sonarLSB = 0;
};

} // namespace OASIS
