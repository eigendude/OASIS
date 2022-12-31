/*
 *  Copyright (C) 2022 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "telemetrix_cpu_fan.hpp"

#include "telemetrix_reports.hpp"

#include <HardwareSerial.h>

using namespace OASIS;

void TelemetrixCPUFan::AttachPWM(uint8_t pwmPin)
{
  m_cpuFanPWM.SetPinModePWM(pwmPin, true);
}

void TelemetrixCPUFan::DetachPWM(uint8_t pwmPin)
{
  m_cpuFanPWM.SetPinModePWM(pwmPin, false);
}

void TelemetrixCPUFan::AttachTachometer(uint8_t tachometerPin)
{
  m_cpuFanTach.SetPinModeTach(tachometerPin, true);
}

void TelemetrixCPUFan::DetachTachometer(uint8_t tachometerPin)
{
  m_cpuFanTach.SetPinModeTach(tachometerPin, false);
}

void TelemetrixCPUFan::PWMWrite(uint8_t digitalPin, float dutyCycle)
{
  m_cpuFanPWM.PWMWrite(digitalPin, dutyCycle);
}

void TelemetrixCPUFan::ScanTachometers()
{
  if (m_tachSamplingInterval == 0)
  {
    // Tachometer reading disabled
    return;
  }

  if (!m_sampleTimer.IsExpired())
  {
    // Timer hasn't elapsed yet
    return;
  }

  m_sampleTimer.SetTimeout(m_tachSamplingInterval);

  // Scan tachometers
  m_cpuFanTach.ScanTachometers(
      [](uint8_t digitalPin, unsigned long rpm)
      {
        //
        // Report message
        //
        // byte 0 = tachometer pin
        // bytes 1-5 = fan speed in RPM
        //
        const uint8_t reportMessage[7] = {6,
                                          CPU_FAN_TACH_REPORT,
                                          static_cast<uint8_t>(digitalPin),
                                          (rpm >> 24) & 0xFF,
                                          (rpm >> 16) & 0xFF,
                                          (rpm >> 8) & 0xFF,
                                          rpm & 0xFF};

        Serial.write(reportMessage, 7);
      });
}

void TelemetrixCPUFan::ResetData()
{
  m_sampleTimer.Reset();
  m_tachSamplingInterval = 0;
}
