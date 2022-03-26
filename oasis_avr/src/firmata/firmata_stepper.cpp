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

#include "firmata_stepper.hpp"

#include <FirmataExpress.h>
#include <Stepper.h>

using namespace OASIS;

void FirmataStepper::Setup(void (*loopFunc)())
{
  // Stepper motor has no loop
}

void FirmataStepper::Reset()
{
  // TODO: Clean up m_stepper
}

void FirmataStepper::Loop()
{
  // Stepper motor has no loop
}
void FirmataStepper::SetStepperPin(uint8_t digitalPin)
{
  Firmata.setPinMode(digitalPin, PIN_MODE_STEPPER);
}

void FirmataStepper::SendStepperLibraryVersion()
{
  if (m_stepper != nullptr)
  {
    const int version = m_stepper->version();

    Firmata.write(START_SYSEX);
    Firmata.write(STEPPER_DATA);
    Firmata.write(version & 0x7F);
    Firmata.write(version >> 7);
    Firmata.write(END_SYSEX);
  }
  else
  {
    // Did not find a configured stepper
    Firmata.sendString("STEPPER FIRMWARE VERSION Error: NO MOTORS CONFIGURED");
  }
}

void FirmataStepper::CreateStepper(int numSteps, int pin1, int pin2)
{
  m_stepper = new Stepper(numSteps, pin1, pin2);
}

void FirmataStepper::CreateStepper(int numSteps, int pin1, int pin2, int pin3, int pin4)
{
  m_stepper = new Stepper(numSteps, pin1, pin2, pin3, pin4);
}

void FirmataStepper::StepMotor(long speed, int numSteps, int direction)
{
  if (m_stepper != nullptr)
  {
    m_stepper->setSpeed(speed);

    if (direction == 0)
      numSteps *= -1;

    m_stepper->step(numSteps);
  }
  else
  {
    Firmata.sendString("STEPPER OPERATE Error: MOTOR NOT CONFIGURED");
  }
}
