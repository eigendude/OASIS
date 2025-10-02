/*
 *  Copyright (C) 2022-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "firmata_cpu_fan.hpp"

#include "firmata_extra.hpp"

#include "arduino_shim.hpp"
#include <FirmataExpress.h>
#include <avr/io.h>

using namespace OASIS;

namespace OASIS
{

// Fan constants
// 0 is fine for most fans, crappy fans may require 10 or 20 to filter out noise
constexpr unsigned int DEBOUNCE_MS = 0;
// If no interrupts were received for 500ms, consider the fan as stuck and
// report 0 RPM
constexpr unsigned int FAN_STUCK_THRESHOLD_MS = 500;

// Utility function
float Clamp(float value, float min, float max)
{
  if (value < min)
    value = min;
  if (value > max)
    value = max;
  return value;
}

} // namespace OASIS

FirmataCPUFan& FirmataCPUFan::GetInstance()
{
  static FirmataCPUFan instance;
  return instance;
}

void FirmataCPUFan::Sample()
{
  if (m_tachometerPin != -1)
  {
    const unsigned long rpm = CalcRPM();

    Firmata.write(START_SYSEX);
    Firmata.write(FIRMATA_CPU_FAN_RPM);

    Firmata.write(static_cast<uint8_t>(m_tachometerPin));

    Firmata.write(static_cast<uint8_t>(rpm & 0x7F));
    Firmata.write(static_cast<uint8_t>((rpm >> 7) & 0x7F));
    Firmata.write(static_cast<uint8_t>((rpm >> 14) & 0x7F));

    Firmata.write(END_SYSEX);
  }
}

bool FirmataCPUFan::SupportsPWM(uint8_t digitalPin)
{
  if (IS_PIN_PWM(digitalPin))
  {
    if (digitalPin == 9 || digitalPin == 10)
      return true;

      // Leonardo undefines timer 2 constants because "iom32u4.h" apparently has
      // the wrong values. See the file "pins_arduino.h" in the Arduino IDE.
#if defined(OCR2A)
    if (digitalPin == 3)
      return true;
#endif
  }

  return false;
}

bool FirmataCPUFan::SupportsTachometer(uint8_t digitalPin)
{
  // TODO: Test if pin is interrupt-capable
  if (digitalPin == 2 || digitalPin == 3)
    return true;

  return false;
}

void FirmataCPUFan::SetPinModePWM(uint8_t digitalPin, bool enable)
{
  if (!SupportsPWM(digitalPin))
    return;

  if (digitalPin == 9 || digitalPin == 10)
  {
    // Enable outputs for Timer 1
    SetupTimer1();
  }
  else if (digitalPin == 3)
  {
    // Enable outputs for Timer 2
    // Note that pin 11 will be unavailable for output in this mode!
    if (!SetupTimer2())
      return;
  }
  else
  {
    // Invalid pin
    return;
  }

  pinMode(digitalPin, OUTPUT);
  Firmata.setPinMode(digitalPin, PIN_MODE_CPU_FAN_PWM);
}

void FirmataCPUFan::SetPinModeTach(uint8_t digitalPin, bool enable)
{
  if (!SupportsTachometer(digitalPin))
    return;

  if (enable)
  {
    // TODO: Support multiple tachometers
    m_tachometerPin = digitalPin;

    // Set the sense pin as input with pullup resistor
    pinMode(digitalPin, INPUT_PULLUP);

    // Set tachISR to be triggered when the signal on the sense pin goes low
    attachInterrupt(digitalPinToInterrupt(digitalPin), StaticTachometerISR, FALLING);

    Firmata.setPinMode(digitalPin, PIN_MODE_CPU_FAN_TACH);
    Firmata.setPinState(digitalPin, 0);
  }
  else
  {
    m_tachometerPin = -1;
    detachInterrupt(digitalPinToInterrupt(digitalPin));
  }
}

void FirmataCPUFan::PWMWrite(uint8_t digitalPin, float dutyCycle)
{
  if (digitalPin == 9)
    SetPWM1A(dutyCycle);
  else if (digitalPin == 10)
    SetPWM1B(dutyCycle);
  else if (digitalPin == 3)
    SetPWM2(dutyCycle);
}

void FirmataCPUFan::SetupTimer1()
{
  // Timer 1 (pins 9, 10) is a high resolution 16 bit timer. This is used by
  // libraries like Servo. We want to take the clock as it is (no prescale) and
  // feed it to the counter; we use mode 10, which counts up to the value of
  // register ICR1, which we set to 320 instead of 65535, giving us a period of
  // roughly 25 kHz.
  //
  // OCR1A and OCR1B control the duty cycle of our output PWM on pins 9 and 10
  // respectively, independently. We have 320 possible values for the duty cycle
  // with this timer.
  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
  TCCR1B = (1 << CS10) | (1 << WGM13);
  ICR1 = 320;
  OCR1A = 0;
  OCR1B = 0;
}

bool FirmataCPUFan::SetupTimer2()
{
  // Timer 2 (pins 3, 11) is a low resolution 8 bit timer. This is used by many
  // libraries and functions such as tone. This timer doesn't have an ICR
  // register like Timer 1, so instead we divide the clock by 8 (prescale 8)
  // and feed it to the counter; we use mode 5 and set output A to trigger a
  // reset of the counter when it reaches 79 instead of 255, resulting in a
  // period of roughly 25 kHz.
  //
  // OCR2B controls the duty cycle of our output PWM on pin 3. Pin 11 is
  // unusable for output because of this mode. We only have 79 possible values
  // for the duty cycle with this timer.
#if defined(OCR2A)
  TIMSK2 = 0;
  TIFR2 = 0;
  TCCR2A = (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);
  TCCR2B = (1 << WGM22) | (1 << CS21);
  OCR2A = 79;
  OCR2B = 0;
  return true;
#endif
  return false;
}

void FirmataCPUFan::SetPWM1A(float dutyCycle)
{
  dutyCycle = Clamp(dutyCycle, 0.0f, 1.0f);
  OCR1A = static_cast<uint16_t>(320 * dutyCycle);
}

void FirmataCPUFan::SetPWM1B(float dutyCycle)
{
  dutyCycle = Clamp(dutyCycle, 0.0f, 1.0f);
  OCR1B = static_cast<uint16_t>(320 * dutyCycle);
}

void FirmataCPUFan::SetPWM2(float dutyCycle)
{
#if defined(OCR2A)
  dutyCycle = Clamp(dutyCycle, 0.0f, 1.0f);
  OCR2B = static_cast<uint16_t>(79 * dutyCycle);
#endif
}

void FirmataCPUFan::StaticTachometerISR()
{
  return GetInstance().TachometerISR();
}

void FirmataCPUFan::TachometerISR()
{
  unsigned long timeMs = millis();
  if (timeMs - m_timestamp2 > DEBOUNCE_MS)
  {
    m_timestamp1 = m_timestamp2;
    m_timestamp2 = timeMs;
  }
}

unsigned long FirmataCPUFan::CalcRPM()
{
  const unsigned long timestamp1 = m_timestamp1;
  const unsigned long timestamp2 = m_timestamp2;

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
