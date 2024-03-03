/*
 *  Copyright (C) 2021-2024 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */
#pragma once

#include "firmata_subsystem.hpp"
#include "utils/timer.hpp"

#include <stdint.h>

namespace OASIS
{

class FirmataDiagnostics : public FirmataSubsystem
{
public:
  // Implementation of FirmataSubsystem
  void Sample() override;

  void ConfigureMemoryReporting(uint32_t reportPeriodMs) { m_reportPeriodMs = reportPeriodMs; }

private:
  // Timing parameters
  Timer m_reportTimer;
  uint32_t m_reportPeriodMs{0};
};

} // namespace OASIS
