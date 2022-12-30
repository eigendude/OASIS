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

#pragma once

#include <stddef.h>
#include <stdint.h>

class Servo;

namespace OASIS
{
class TelemetrixServo
{
public:
  TelemetrixServo();

  void servo_attach(uint8_t pin, int minpulse, int maxpulse);
  void servo_write(uint8_t pin, int angle);
  void servo_detach(uint8_t pin);

  void reset_data();

private:
  // Find the first servo that is not attached to a pin. This is a helper
  // function not called directly via the API.
  int find_first_servo();

  // Servo management, array of size MAX_SERVOS
  Servo* const m_servos;

  // This array of size MAX_SERVOS allows us to retrieve the servo object
  // associated with a specific pin number
  uint8_t* const m_pinToServoIndexMap;
};
} // namespace OASIS
