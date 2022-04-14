/*
 *  Copyright (C) 2022 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "leds/heartbeat_thread.hpp"
#include "telemetrix/telemetrix_server.hpp"

#include <Arduino.h>
#include <Scheduler.h>

using namespace OASIS;

void setup()
{
  // Initialize LEDs
  HeartbeatThread::GetInstance().Setup();

  // TODO: Initialize Bluetooth

  // Initialize Telemetrix (also initializes serial)
  telemetrix_setup();
}

void loop()
{
  // Loop telemetrix
  telemetrix_loop();

  // Run queued off-thread tasks
  yield();
}
