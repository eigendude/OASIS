/*
 *  Copyright (C) 2021-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "heartbeat_thread.hpp"

#include <Arduino.h>

#include "firmata/firmata_scheduler.hpp"

using namespace OASIS;

namespace OASIS
{

// LED parameters
constexpr unsigned int HEARTBEAT_LED = LED_BUILTIN;

// Instance storage
HeartbeatThread heartbeatInstance;

} // namespace OASIS

HeartbeatThread& HeartbeatThread::GetInstance()
{
  return heartbeatInstance;
}

void HeartbeatThread::Setup()
{
  // Setup to blink the inbuilt LED
  pinMode(HEARTBEAT_LED, OUTPUT);

  InitializeTaskScheduler();

  static TsTask heartbeatTask(TASK_IMMEDIATE, TASK_FOREVER, HeartbeatLoop);
  GetTaskScheduler().addTask(heartbeatTask);
  heartbeatTask.enable();
}

void HeartbeatThread::Loop()
{
  digitalWrite(HEARTBEAT_LED, HIGH);

  delay(100);

  digitalWrite(HEARTBEAT_LED, LOW);

  delay(150);

  digitalWrite(HEARTBEAT_LED, HIGH);

  delay(100);

  digitalWrite(HEARTBEAT_LED, LOW);

  delay(800);
}

void HeartbeatThread::HeartbeatLoop()
{
  GetInstance().Loop();
}
