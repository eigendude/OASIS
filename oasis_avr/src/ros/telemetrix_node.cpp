/*
 *  Copyright (C) 2022 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "leds/heartbeat_thread.hpp"
#include "telemetrix/telemetrix_server.hpp"

#if defined(BLUEFRUIT_ENABLED)
#include "drivers/bluefruit.hpp"
#endif

#include <stdint.h>

#include <Arduino.h>
#include <Scheduler.h>

using namespace OASIS;

namespace
{

#if defined(BLUEFRUIT_ENABLED)
// Bluetooth parameters
Bluefruit bluefruit;
#endif

} // namespace

void setup()
{
  // Initialize LEDs
  HeartbeatThread::GetInstance().Setup();

#if defined(BLUEFRUIT_ENABLED)
  // Initialize Bluetooth
  bluefruit.Setup();
#endif

  // Initialize Telemetrix (also initializes serial)
  telemetrix_setup();
}

void loop()
{
#if defined(BLUEFRUIT_ENABLED)
  // Loop Bluetooth
  bluefruit.Loop();
#endif

  // Loop telemetrix
#if defined(BLUEFRUIT_ENABLED)
  telemetrix_loop(&bluefruit);
#else
  telemetrix_loop(nullptr);
#endif

  // Run queued off-thread tasks
  yield();
}
