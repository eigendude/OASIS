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
#pragma once

#include "firmata_subsystem.hpp"

#include <stdint.h>

#include <Boards.h>
#include <Servo.h>

namespace OASIS
{

class FirmataServo : public FirmataSubsystem
{
public:
  // Servo functions
  uint8_t GetServoPin(uint8_t digitalPin) const;
  void PrepareServoPin(uint8_t digitalPin);
  bool IsServoAttached(uint8_t digitalPin) const;
  bool AttachServo(uint8_t digitalPin, int minPulse, int maxPulse);
  void DetachServo(uint8_t digitalPin);
  void SetServoMode(uint8_t digitalPin);
  void WriteServo(uint8_t digitalPin, int analogValue);

private:
  // Servo state
  Servo m_servos[MAX_SERVOS]{};
  uint8_t m_servoPinMap[TOTAL_PINS]{};
  uint8_t m_detachedServos[MAX_SERVOS]{};
  uint8_t m_detachedServoCount{0};
  uint8_t m_servoCount{0};
};

} // namespace OASIS
