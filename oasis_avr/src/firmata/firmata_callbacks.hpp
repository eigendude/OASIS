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
#pragma once

#include <stdint.h>

namespace OASIS
{

class FirmataThread;

class FirmataCallbacks
{
public:
  /*!
   * \brief Initialize the Firmata callbacks subsystem
   */
  static void InitializeCallbacks(FirmataThread& thread);

  static void PWMWriteCallback(uint8_t analogPin, int analogValue);

  static void DigitalWriteCallback(uint8_t digitalPort, int portValue);

  /*!
   * \brief Sets bits in a bit array (int) to toggle the reporting of the
   * analogIns
   */
  static void ReportAnalogCallback(uint8_t analogPin, int enableReporting);

  static void ReportDigitalCallback(uint8_t digitalPort, int enableReporting);

  /*!
   * \brief Sets the pin mode to the correct state and sets the relevant bits in
   * the two bit-arrays that track Digital I/O and PWM status
   */
  static void SetPinModeCallback(uint8_t pin, int mode);

  /*!
   * \brief Sets the value of an individual pin
   *
   * Useful if you want to set a pin value but are not tracking the digital port
   * state.
   *
   * Can only be used on pins configured as OUTPUT. Cannot be used to enable
   * pull-ups on Digital INPUT pins.
   */
  static void SetPinValueCallback(uint8_t pin, int value);

  static void SysexCallback(uint8_t command, uint8_t argc, uint8_t* argv);

  static void SystemResetCallback();

private:
  // Firmata parameters
  static FirmataThread* m_thread;
};

} // namespace OASIS
