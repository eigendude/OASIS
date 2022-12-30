/*
 *  Copyright (C) 2022 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  This file is derived from Telemetrix4Arduino under the AGPL 3.0 License
 *  Copyright (c) 2020-2021 Alan Yorinks
 *
 *  SPDX-License-Identifier: Apache-2.0 AND AGPL-3.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "telemetrix_servo.hpp"

#include "telemetrix_reports.hpp"

#include <HardwareSerial.h>
#include <Servo.h>

using namespace OASIS;

namespace
{
static Servo servos[MAX_SERVOS];
static uint8_t pinToServoIndexMap[MAX_SERVOS];
} // namespace

TelemetrixServo::TelemetrixServo() : m_servos(servos), m_pinToServoIndexMap(pinToServoIndexMap)
{
}

void TelemetrixServo::servo_attach(uint8_t pin, int minpulse, int maxpulse)
{
  // Find the first available open servo
  const int servoFound = find_first_servo();
  if (servoFound != -1)
  {
    m_pinToServoIndexMap[servoFound] = pin;
    m_servos[servoFound].attach(pin, minpulse, maxpulse);
  }
  else
  {
    // No open servos available, send a report back to client
    const uint8_t reportMessage[2] = {SERVO_UNAVAILABLE, pin};
    Serial.write(reportMessage, 2);
  }
}

void TelemetrixServo::servo_write(uint8_t pin, int angle)
{
  // Find the servo object for the pin
  for (unsigned int i = 0; i < MAX_SERVOS; ++i)
  {
    if (m_pinToServoIndexMap[i] == pin)
    {
      m_servos[i].write(angle);
      return;
    }
  }
}

void TelemetrixServo::servo_detach(uint8_t pin)
{
  // Find the servo object for the pin
  for (unsigned int i = 0; i < MAX_SERVOS; ++i)
  {
    if (m_pinToServoIndexMap[i] == pin)
    {
      m_pinToServoIndexMap[i] = -1;
      m_servos[i].detach();
    }
  }
}

void TelemetrixServo::reset_data()
{
  // Detach any attached servos
  for (unsigned int i = 0; i < MAX_SERVOS; ++i)
  {
    if (m_servos[i].attached())
      m_servos[i].detach();
  }
}

int TelemetrixServo::find_first_servo()
{
  int index = -1;

  for (unsigned int i = 0; i < MAX_SERVOS; ++i)
  {
    if (!m_servos[i].attached())
    {
      index = i;
      break;
    }
  }

  return index;
}
