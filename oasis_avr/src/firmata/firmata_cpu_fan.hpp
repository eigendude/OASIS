/*
 *  Copyright (C) 2022-2024 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */
#pragma once

#include "firmata_subsystem.hpp"

#include <stdint.h>

namespace OASIS
{

/*!
 * \brief Firmata subsystem to control a CPU fan
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
 *
 * For RPM detection, the speed sense wire is expected to be connected with a
 * pullup resistor and it generates 2 impulses per rotation. The best way to
 * read this is to use one of the Arduino pins that can do interrupts (pins 2
 * and 3 on the Arduino Uno).
 *
 * TODO: Support multiple tachometers
 */
class FirmataCPUFan : public FirmataSubsystem
{
public:
  // Implementation of FirmataSubsystem
  void Sample() override;

  // CPU fan functions
  bool SupportsPWM(uint8_t digitalPin);
  bool SupportsTachometer(uint8_t digitalPin);
  void SetPinModePWM(uint8_t digitalPin, bool enable);
  void SetPinModeTach(uint8_t digitalPin, bool enable);
  void PWMWrite(uint8_t digitalPin, float dutyCycle);

private:
  // TODO: Support multiple tachometers
  static FirmataCPUFan& GetInstance();

  // Configure Timer 1 (pins 9, 10) to output 25kHz PWM
  void SetupTimer1();

  // Configure Timer 2 (pin 3) to output 25kHz PWM. Pin 11 will be unavailable
  // for output in this mode
  bool SetupTimer2();

  // Equivalent of analogWrite on pin 9
  void SetPWM1A(float dutyCycle);

  // Equivalent of analogWrite on pin 10
  void SetPWM1B(float dutyCycle);

  // Equivalent of analogWrite on pin 3
  void SetPWM2(float dutyCycle);

  // Interrupt handler. Stores the timestamps of the last 2 interrupts and
  // handles debouncing.
  static void StaticTachometerISR();
  void TachometerISR();

  // Calculates the RPM based on the timestamps of the last 2 interrupts. Can
  // be called at any time.
  unsigned long CalcRPM();

  int m_tachometerPin{-1}; // -1 if no tachometer is enabled
  unsigned long volatile m_timestamp1{0};
  unsigned long volatile m_timestamp2{0};
};

} // namespace OASIS
