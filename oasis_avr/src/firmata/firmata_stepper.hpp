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

#include <stdint.h>

class Stepper;

namespace OASIS
{

// Stepper motor constants
static constexpr uint8_t STEPPER_COMMAND_CONFIGURE = 0;
static constexpr uint8_t STEPPER_COMMAND_STEP = 1;
static constexpr uint8_t STEPPER_COMMAND_LIBRARY_VERSION = 2;

class FirmataStepper : public FirmataSubsystem
{
public:
  // Stepper motor functions
  void SetStepperPin(uint8_t digitalPin);
  void SendStepperLibraryVersion();
  void CreateStepper(int numSteps, int pin1, int pin2);
  void CreateStepper(int numSteps, int pin1, int pin2, int pin3, int pin4);
  void StepMotor(long speed, int numSteps, int direction);

private:
  // Stepper motor state
  Stepper* m_stepper = nullptr;
};

} // namespace OASIS
