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
  HeartbeatThread::GetInstance().Setup();
  TelemetrixThread::GetInstance().Setup();
}

void loop()
{
  TelemetrixThread::GetInstance().Loop();
  RunTaskScheduler();
}
