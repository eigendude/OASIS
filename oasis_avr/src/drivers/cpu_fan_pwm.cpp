/*
 *  Copyright (C) 2022-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "cpu_fan_pwm.hpp"

#include "utils/math_utils.hpp"

#include <Arduino.h>
#include <avr/io.h>

using namespace OASIS;

bool CPUFanPWM::SupportsPWM(uint8_t digitalPin)
{
  if (digitalPin == 9 || digitalPin == 10)
    return true;

    // Leonardo undefines timer 2 constants because "iom32u4.h" apparently has
    // the wrong values. See the file "pins_arduino.h" in the Arduino IDE.
#if defined(OCR2A)
  if (digitalPin == 3)
    return true;
#endif

  return false;
}

void CPUFanPWM::SetPinModePWM(uint8_t digitalPin, bool enable)
{
  if (!SupportsPWM(digitalPin))
    return;

  if (enable)
  {
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
  }
  else
  {
    // TODO: Allow disabling PWM mode
  }

  pinMode(digitalPin, OUTPUT);
}

void CPUFanPWM::PWMWrite(uint8_t digitalPin, float dutyCycle)
{
  if (digitalPin == 9)
    SetPWM1A(dutyCycle);
  else if (digitalPin == 10)
    SetPWM1B(dutyCycle);
  else if (digitalPin == 3)
    SetPWM2(dutyCycle);
}

void CPUFanPWM::SetupTimer1()
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

bool CPUFanPWM::SetupTimer2()
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

void CPUFanPWM::SetPWM1A(float dutyCycle)
{
  dutyCycle = MathUtils::Clamp(dutyCycle, 0.0f, 1.0f);

  OCR1A = static_cast<uint16_t>(320 * dutyCycle);
}

void CPUFanPWM::SetPWM1B(float dutyCycle)
{
  dutyCycle = MathUtils::Clamp(dutyCycle, 0.0f, 1.0f);

  OCR1B = static_cast<uint16_t>(320 * dutyCycle);
}

void CPUFanPWM::SetPWM2(float dutyCycle)
{
#if defined(OCR2A)
  dutyCycle = MathUtils::Clamp(dutyCycle, 0.0f, 1.0f);

  OCR2B = static_cast<uint16_t>(79 * dutyCycle);
#endif
}
