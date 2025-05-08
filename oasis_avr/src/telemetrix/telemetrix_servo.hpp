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

#pragma once

#include <stdint.h>

namespace OASIS
{
class Servomotor;

class TelemetrixServo
{
public:
  void ServoAttach(uint8_t pin, int minpulse, int maxpulse);
  void ServoDetach(uint8_t pin);

  void ServoWrite(uint8_t pin, int angle);

  void ResetData();

private:
  // Find the first servo that is not attached to a pin. This is a helper
  // function not called directly via the API.
  int FindFirstServo();

  // Servo management, array of size MAX_SERVOS
  Servomotor* m_servos{nullptr};

  // This array of size MAX_SERVOS allows us to retrieve the servo object
  // associated with a specific pin number
  uint8_t* m_pinToServoIndexMap{nullptr};
};
} // namespace OASIS
