/*
 *  Copyright (C) 2021-2023 Garrett Brown
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
constexpr size_t HEARTBEAT_STACK_SIZE = 8; // Default is 128

// LED parameters
constexpr unsigned int HEARTBEAT_LED = LED_BUILTIN;

// Instance storage
HeartbeatThread instance;

} // namespace OASIS

HeartbeatThread& HeartbeatThread::GetInstance()
{
  return instance;
}

void HeartbeatThread::Setup()
{
  // Setup to blink the inbuilt LED
  pinMode(HEARTBEAT_LED, OUTPUT);

  // Start heartbeat thread
  Scheduler.startLoop(HeartbeatLoop, HEARTBEAT_STACK_SIZE);
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
