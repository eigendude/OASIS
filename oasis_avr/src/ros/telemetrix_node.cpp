/*
 *  Copyright (C) 2022-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "leds/heartbeat_thread.hpp"
#include "telemetrix/telemetrix_server.hpp"

#include <stdint.h>

#include <Arduino.h>
#include <Scheduler.h>

using namespace OASIS;

namespace
{
static TelemetrixServer telemetrixServer;
}

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
  yield();
}
