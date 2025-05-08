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
 * \brief Interface to read a CPU fan tachometer
 *
 * For RPM detection, the speed sense wire is expected to be connected with a
 * pullup resistor and it generates 2 impulses per rotation. The best way to
 * read this is to use one of the Arduino pins that can do interrupts (pins 2
 * and 3 on the Arduino Uno).
 */
class CPUFanTach
{
public:
  static bool SupportsTachometer(uint8_t digitalPin);

  void SetPinModeTach(uint8_t digitalPin, bool enable);

  void ScanTachometers(void (*scanCallback)(uint8_t digitalPin, unsigned long rpm));

private:
  using ISRType = void (*)();

  struct CPUFanTachPin
  {
    const uint8_t digitalPin;
    unsigned long volatile timestamp1{0};
    unsigned long volatile timestamp2{0};
  };

  // Global instance to dereference static ISRs
  static CPUFanTach& GetInstance();

  // Get the ISR index of a digital pin, or -1 if there is no ISR for the pin
  static int GetISRIndex(uint8_t digitalPin);

  /*!
   * \brief Interrupt handlers. We need a static function for each tach up to
   * MAX_CPU_FAN_TACHOMETER.
   */
  static void StaticTachometerISR0() { return GetInstance().TachometerISR(0); }
  static void StaticTachometerISR1() { return GetInstance().TachometerISR(1); }

  static constexpr ISRType ISRS[] = {StaticTachometerISR0, StaticTachometerISR1};
  static constexpr uint8_t ISR_COUNT = sizeof(ISRS) / sizeof(*ISRS);

  /*!
   * \brief Updates the timestamps of the last 2 interrupts and handles
   * debouncing
   */
  void TachometerISR(uint8_t isrIndex);

  /*!
   * \brief Calculates the RPM based on the timestamps of the last 2 interrupts
   */
  static unsigned long CalcRPM(unsigned long timestamp1, unsigned long timestamp2);

  // Tachometer state
  CPUFanTachPin* m_tachometers[ISR_COUNT];
};
} // namespace OASIS
