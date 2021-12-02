/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "firmata/firmata_thread.hpp"
#include "leds/heartbeat_thread.hpp"

#include <Arduino.h>
#include <Scheduler.h>

using namespace OASIS;

namespace OASIS
{

constexpr uint8_t LED_PIN_RED = 10;
constexpr uint8_t LED_PIN_GREEN = 11;

constexpr size_t startupAnimationStackSize = 32; // Default is 128

// Startup animation state
static bool started = false;
static int brightness = 0;

void StartupAnimationSetup()
{
  // Setup the PWM LEDs
  pinMode(LED_PIN_RED, OUTPUT);
  pinMode(LED_PIN_GREEN, OUTPUT);
}

void StartupAnimationLoop()
{
  if (!started)
  {
    for (; brightness < 250; brightness += 10)
    {
      analogWrite(LED_PIN_GREEN, brightness);
      analogWrite(LED_PIN_RED, brightness);
      delay(brightness > 100 ? 15 : 40);
    }

    for (; brightness >= 0; brightness -= 10)
    {
      analogWrite(LED_PIN_GREEN, brightness);
      analogWrite(LED_PIN_RED, brightness);
      delay(brightness > 100 ? 15 : 40);
    }

    // Update state
    started = true;
  }

  yield();
}

} // namespace OASIS

void setup()
{
  HeartbeatThread::GetInstance().Setup();
  FirmataThread::GetInstance().Setup();

  Scheduler.start(StartupAnimationSetup, StartupAnimationLoop, startupAnimationStackSize);
}

void loop()
{
  // TODO: Delay forever
  delay(1000);
}
