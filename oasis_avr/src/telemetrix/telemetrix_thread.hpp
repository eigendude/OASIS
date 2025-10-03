/*
 *  Copyright (C) 2022-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */
#pragma once

#include "telemetrix_server.hpp"

namespace OASIS
{

class TelemetrixThread
{
public:
  static TelemetrixThread& GetInstance();

  void Setup();

private:
  TelemetrixThread() = default;

  // Task entry points
  void Loop();
  static void TelemetrixLoop();

  TelemetrixServer m_server;
};

} // namespace OASIS
