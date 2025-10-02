/*
 *  Copyright (C) 2021-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "firmata/firmata_thread.hpp"
#include "leds/heartbeat_thread.hpp"

#include <Arduino.h>

#include "firmata/firmata_scheduler.hpp"

using namespace OASIS;

void setup()
{
  HeartbeatThread::GetInstance().Setup();
  FirmataThread::GetInstance().Setup();
}

void loop()
{
  RunTaskScheduler();
}
