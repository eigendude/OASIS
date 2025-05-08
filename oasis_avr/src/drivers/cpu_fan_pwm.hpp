/*
 *  Copyright (C) 2022-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */
#pragma once

#include <stdint.h>

namespace OASIS
{

/*!
 * \brief Interface to control a 4-wire CPU fan's PWM signal
 *
 * CPU fan control is kind of tricky. The specs require a PWM signal with a
 * frequency of 25 kHz (with tolerance, 21-18 kHz), but our usual analogWrite
 * function doesn't output anywhere near that frequency.
 *
 * By using some timer tricks, we can make pins 9, 10 and 3 generate signals
 * at the correct frequency.
 *
 * Note that when a CPU fan is connected to pin 3, pin 11 is unavailable for
 * output.
 *
 * Reusable timer handling is from https://fdossena.com/?p=ArduinoFanControl/i.md.
 */
class CPUFanPWM
{
public:
  /*!
	 * \brief Check if the specified pin can control a CPU fan PWM signal
	 */
  static bool SupportsPWM(uint8_t digitalPin);

  /*!
	 * \brief Set pin mode to PWM fan control
	 *
	 * Note that "unsetting" a PWM pin is not implemented, you'll have to
	 * restart the Arduino
	 */
  void SetPinModePWM(uint8_t digitalPin, bool enable);

  /*!
   * \brief Write an analog value to a PWM pin controlling a CPU fan
   */
  void PWMWrite(uint8_t digitalPin, float dutyCycle);

private:
  /*!
   * \brief Configure Timer 1 (pins 9, 10) to output 25kHz PWM
   */
  void SetupTimer1();

  /*!
   * \brief Configure Timer 2 (pin 3) to output 25kHz PWM
   *
   * Note that pin 11 will be unavailable for output in this mode.
   */
  bool SetupTimer2();

  /*!
   * \brief Equivalent of analogWrite on pin 9
   */
  void SetPWM1A(float dutyCycle);

  /*!
   * \brief Equivalent of analogWrite on pin 10
   */
  void SetPWM1B(float dutyCycle);

  /*!
   * \brief Equivalent of analogWrite on pin 3
   */
  void SetPWM2(float dutyCycle);
};
} // namespace OASIS
