/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "telemetrix_thread.hpp"

#include "telemetrix/telemetrix_server.hpp"

using namespace OASIS;

TelemetrixThread& TelemetrixThread::GetInstance()
{
  static TelemetrixThread instance;
  return instance;
}

void TelemetrixThread::Setup()
{
  m_server.Setup();
}

void TelemetrixThread::Loop()
{
  m_server.Loop();
}
