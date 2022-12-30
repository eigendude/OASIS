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

#include "utils/timer.hpp"

#include <stdint.h>

namespace OASIS
{
class TelemetrixMemory
{
public:
  void SetReportingInterval(uint32_t intervalMs) { m_reportIntervalMs = intervalMs; }

  void ScanMemory();

  void ResetData();

private:
  // Timing parameters
  Timer m_reportTimer;
  uint32_t m_reportIntervalMs{0};
};
} // namespace OASIS
