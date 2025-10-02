/*
 *  Copyright (C) 2022-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "cpu_fan_tach.hpp"

#include "arduino_shim.hpp"

using namespace OASIS;

namespace
{
// 0 is fine for most fans, crappy fans may require 10 or 20 to filter out noise
constexpr unsigned int DEBOUNCE_MS = 0;

// If no interrupts were received for 500ms, consider the fan as stuck and
// report 0 RPM
constexpr unsigned int FAN_STUCK_THRESHOLD_MS = 500;
} // namespace

CPUFanTach& CPUFanTach::GetInstance()
{
  static CPUFanTach instance;
  return instance;
}

bool CPUFanTach::SupportsTachometer(uint8_t digitalPin)
{
  return GetISRIndex(digitalPin) >= 0;
}

void CPUFanTach::SetPinModeTach(uint8_t digitalPin, bool enable)
{
  const int isrIndex = GetISRIndex(digitalPin);
  if (isrIndex < 0)
  {
    // Pin is not interrupt-capable
    return;
  }

  if (enable && m_tachometers[isrIndex] == nullptr)
  {
    m_tachometers[isrIndex] = new CPUFanTachPin{digitalPin};

    // Set the sense pin as input with pullup resistor
    pinMode(digitalPin, INPUT_PULLUP);

    // Set tachISR to be triggered when the signal on the sense pin goes low
    attachInterrupt(digitalPinToInterrupt(digitalPin), ISRS[isrIndex], FALLING);
  }
  else if (!enable && m_tachometers[isrIndex] != nullptr)
  {
    delete m_tachometers[isrIndex];
    m_tachometers[isrIndex] = nullptr;

    detachInterrupt(digitalPinToInterrupt(digitalPin));
  }
}

void CPUFanTach::ScanTachometers(void (*scanCallback)(uint8_t digitalPin, unsigned long rpm))
{
  for (unsigned int i = 0; i < ISR_COUNT; ++i)
  {
    const CPUFanTachPin* tachPin = m_tachometers[i];
    if (tachPin != nullptr)
      scanCallback(tachPin->digitalPin, CalcRPM(tachPin->timestamp1, tachPin->timestamp2));
  }
}

int CPUFanTach::GetISRIndex(uint8_t digitalPin)
{
  // TODO: Test if pin is interrupt-capable
  switch (digitalPin)
  {
    case 2:
      return 0;
    case 3:
      return 1;
    default:
      break;
  }
  return -1;
}

void CPUFanTach::TachometerISR(uint8_t isrIndex)
{
  CPUFanTachPin* tachometer = m_tachometers[isrIndex];
  if (tachometer != nullptr)
  {
    const unsigned long timeMs = millis();
    if (timeMs - tachometer->timestamp2 > DEBOUNCE_MS)
    {
      tachometer->timestamp1 = tachometer->timestamp2;
      tachometer->timestamp2 = timeMs;
    }
  }
}

unsigned long CPUFanTach::CalcRPM(unsigned long timestamp1, unsigned long timestamp2)
{
  // Check if ISR hasn't been invoked
  if (timestamp1 == 0 || timestamp2 == 0)
    return 0;

  // Check for infinite speed
  if (timestamp1 == timestamp2)
    return 0;

  // Check for stuck fan
  if (millis() - timestamp2 >= FAN_STUCK_THRESHOLD_MS)
    return 0;

  return (60000 / (timestamp2 - timestamp1)) / 2;
}
