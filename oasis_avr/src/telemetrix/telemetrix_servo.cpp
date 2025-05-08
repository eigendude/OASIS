/*
 *  Copyright (C) 2022-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  This file is derived from Telemetrix4Arduino under the AGPL 3.0 License
 *  Copyright (c) 2020-2021 Alan Yorinks
 *
 *  SPDX-License-Identifier: Apache-2.0 AND AGPL-3.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "telemetrix_servo.hpp"

#include "drivers/servomotor.hpp"
#include "telemetrix_reports.hpp"

#include <HardwareSerial.h>

using namespace OASIS;

void TelemetrixServo::ServoAttach(uint8_t pin, int minpulse, int maxpulse)
{
  if (m_servos == nullptr)
    m_servos = new Servomotor[MAX_SERVOS];

  if (m_pinToServoIndexMap == nullptr)
    m_pinToServoIndexMap = new uint8_t[MAX_SERVOS];

  // Find the first available open servo
  const int servoFound = FindFirstServo();
  if (servoFound != -1)
  {
    m_pinToServoIndexMap[servoFound] = pin;
    m_servos[servoFound].Attach(pin, minpulse, maxpulse);
  }
  else
  {
    // No open servos available, send a report back to client
    const uint8_t reportMessage[2] = {SERVO_UNAVAILABLE, pin};
    Serial.write(reportMessage, 2);
  }
}

void TelemetrixServo::ServoWrite(uint8_t pin, int angle)
{
  // Find the servo object for the pin
  for (unsigned int i = 0; i < MAX_SERVOS; ++i)
  {
    if (m_pinToServoIndexMap[i] == pin)
    {
      m_servos[i].Write(angle);
      return;
    }
  }
}

void TelemetrixServo::ServoDetach(uint8_t pin)
{
  // Find the servo object for the pin
  for (unsigned int i = 0; i < MAX_SERVOS; ++i)
  {
    if (m_pinToServoIndexMap[i] == pin)
    {
      m_pinToServoIndexMap[i] = -1;
      m_servos[i].Detach();
    }
  }
}

void TelemetrixServo::ResetData()
{
  // Detach any attached servos
  for (unsigned int i = 0; i < MAX_SERVOS; ++i)
  {
    if (m_servos[i].Attached())
      m_servos[i].Detach();
  }
}

int TelemetrixServo::FindFirstServo()
{
  int index = -1;

  for (unsigned int i = 0; i < MAX_SERVOS; ++i)
  {
    if (!m_servos[i].Attached())
    {
      index = i;
      break;
    }
  }

  return index;
}
