/*
 *  Copyright (C) 2022-2024 Garrett Brown
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

#define MAX_SONARS 6

namespace OASIS
{
class Sonar;

class TelemetrixSonar
{
public:
  void AttachSonar(uint8_t triggerPin, uint8_t echoPin);

  void ScanSonars();

  void ResetData();

private:
  // Get the next available sonar index
  int GetSonarIndex() const;

  // An array of sonar objects
  Sonar* m_sonars[MAX_SONARS]{};
};
} // namespace OASIS
