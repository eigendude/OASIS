/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "heartbeat_thread.hpp"

#include <Arduino.h>
#include <Scheduler.h>

using namespace OASIS;

namespace OASIS
{

// Threading constants
constexpr size_t HEARTBEAT_STACK_SIZE = 96; // Default is 128

} // namespace OASIS

HeartbeatThread& HeartbeatThread::GetInstance()
{
  // Instance storage
  static HeartbeatThread instance;

  return instance;
}

void HeartbeatThread::Setup()
{
  // Setup to blink the inbuilt LED
  pinMode(LED_BUILTIN, OUTPUT);

  // Start heartbeat thread
  Scheduler.startLoop(HeartbeatLoop, HEARTBEAT_STACK_SIZE);
}

void HeartbeatThread::Loop()
{
  digitalWrite(LED_BUILTIN, HIGH);

  delay(100);

  digitalWrite(LED_BUILTIN, LOW);

  delay(150);

  digitalWrite(LED_BUILTIN, HIGH);

  delay(100);

  digitalWrite(LED_BUILTIN, LOW);

  delay(800);
}

void HeartbeatThread::HeartbeatLoop()
{
  GetInstance().Loop();
}
