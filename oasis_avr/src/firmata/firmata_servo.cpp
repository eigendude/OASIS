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

#include "firmata_servo.hpp"

#include <FirmataExpress.h>
#include <Scheduler.h>

using namespace OASIS;

uint8_t FirmataServo::GetServoPin(uint8_t digitalPin) const
{
  return m_servoPinMap[digitalPin];
}

void FirmataServo::PrepareServoPin(uint8_t digitalPin)
{
  if (m_servoPinMap[digitalPin] < MAX_SERVOS && m_servos[m_servoPinMap[digitalPin]].attached())
    DetachServo(digitalPin);
}

bool FirmataServo::IsServoAttached(uint8_t digitalPin) const
{
  Servo& servo = const_cast<Servo&>(m_servos[m_servoPinMap[digitalPin]]);

  return servo.attached();
}

bool FirmataServo::AttachServo(uint8_t digitalPin, int minPulse, int maxPulse)
{
  if (GetServoPin(digitalPin) < MAX_SERVOS && IsServoAttached(digitalPin))
    DetachServo(digitalPin);

  if (m_servoCount < MAX_SERVOS)
  {
    // Reuse indexes of detached servos until all have been reallocated
    if (m_detachedServoCount > 0)
    {
      m_servoPinMap[digitalPin] = m_detachedServos[m_detachedServoCount - 1];
      if (m_detachedServoCount > 0)
        m_detachedServoCount--;
    }
    else
    {
      m_servoPinMap[digitalPin] = m_servoCount;
      m_servoCount++;
    }

    if (minPulse > 0 && maxPulse > 0)
      m_servos[m_servoPinMap[digitalPin]].attach(PIN_TO_DIGITAL(digitalPin), minPulse, maxPulse);
    else
      m_servos[m_servoPinMap[digitalPin]].attach(PIN_TO_DIGITAL(digitalPin));
  }
  else
  {
    Firmata.sendString("Max servos attached");
    return false;
  }

  return true;
}

void FirmataServo::DetachServo(uint8_t digitalPin)
{
  m_servos[m_servoPinMap[digitalPin]].detach();

  // If we're detaching the last servo, decrement the count. Otherwise store
  // the index of the detached servo.
  if (m_servoPinMap[digitalPin] == m_servoCount && m_servoCount > 0)
  {
    m_servoCount--;
  }
  else if (m_servoCount > 0)
  {
    // Keep track of detached servos because we want to reuse their indexes
    // before incrementing the count of attached servos
    m_detachedServoCount++;
    m_detachedServos[m_detachedServoCount - 1] = m_servoPinMap[digitalPin];
  }

  m_servoPinMap[digitalPin] = 255;
}

void FirmataServo::SetServoMode(uint8_t digitalPin)
{
  if (m_servoPinMap[digitalPin] == 255 || !m_servos[m_servoPinMap[digitalPin]].attached())
  {
    // Pass -1 for min and max pulse values to use default values set by
    // Servo library
    AttachServo(digitalPin, -1, -1);

    Firmata.setPinMode(digitalPin, PIN_MODE_SERVO);
  }
}

void FirmataServo::WriteServo(uint8_t pin, int analogValue)
{
  m_servos[m_servoPinMap[pin]].write(analogValue);

  Firmata.setPinState(pin, analogValue);
}
