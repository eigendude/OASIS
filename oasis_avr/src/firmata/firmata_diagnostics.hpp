/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */
#pragma once

#include <stdint.h>

namespace OASIS
{

class FirmataDiagnostics
{
public:
  // Lifecycle functions
  void Setup(void (*loopFunc)());
  void Reset();
  void Loop();

  void ConfigureMemoryReporting(uint32_t reportPeriodMs) { m_reportPeriodMs = reportPeriodMs; }

private:
  uint32_t m_reportPeriodMs = 0;
};

} // namespace OASIS
