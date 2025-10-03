/*
 *  Copyright (C) 2022-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "leds/heartbeat_task.hpp"
#include "scheduler/task_scheduler.hpp"
#include "telemetrix/telemetrix_thread.hpp"

using namespace OASIS;

void setup()
{
  HeartbeatTask::GetInstance().Setup();
  TelemetrixThread::GetInstance().Setup();
}

void loop()
{
  RunTaskScheduler();
}
