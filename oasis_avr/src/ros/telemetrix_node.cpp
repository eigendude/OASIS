/*
 *  Copyright (C) 2022-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "leds/heartbeat_thread.hpp"
#include "scheduler/task_scheduler.hpp"
#include "telemetrix/telemetrix_server.hpp"

#include <Arduino.h>

using namespace OASIS;

namespace
{
TelemetrixServer telemetrixServer;
} // namespace

void setup()
{
  // Initialize LEDs
  HeartbeatThread::GetInstance().Setup();

  // Initialize Telemetrix (also initializes serial)
  telemetrixServer.Setup();
}

void loop()
{
  // Loop Telemetrix
  telemetrixServer.Loop();

  // Run queued off-thread tasks
  RunTaskScheduler();
}
