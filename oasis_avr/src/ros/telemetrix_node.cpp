/*
 *  Copyright (C) 2022-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "leds/heartbeat_thread.hpp"
#include "scheduler/task_scheduler.hpp"
#include "telemetrix/telemetrix_thread.hpp"

#include <Arduino.h>

using namespace OASIS;

void setup()
{
  // Initialize LEDs
  HeartbeatThread::GetInstance().Setup();

  // Initialize Telemetrix (also initializes serial)
  TelemetrixThread::GetInstance().Setup();
}

void loop()
{
  RunTaskScheduler();
}
