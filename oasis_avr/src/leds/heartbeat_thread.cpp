/*
 *  Copyright (C) 2021-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "heartbeat_thread.hpp"

#include "firmata/firmata_scheduler.hpp"

#include <Arduino.h>
#include <TScheduler.hpp>

using namespace OASIS;

namespace OASIS
{

// LED parameters
constexpr unsigned int HEARTBEAT_LED = LED_BUILTIN;

// Instance storage
HeartbeatThread heartbeatInstance;

struct HeartbeatPhase
{
  bool levelHigh;
  unsigned long durationMs;
};

constexpr HeartbeatPhase HEARTBEAT_SEQUENCE[] = {
    {true, 100}, // First pulse
    {false, 150}, // Short pause
    {true, 100}, // Second pulse
    {false, 800}, // Long pause
};

constexpr unsigned int HEARTBEAT_PHASE_COUNT =
    sizeof(HEARTBEAT_SEQUENCE) / sizeof(HEARTBEAT_SEQUENCE[0]);

} // namespace OASIS

HeartbeatThread& HeartbeatThread::GetInstance()
{
  return heartbeatInstance;
}

void HeartbeatThread::Setup()
{
  // Setup to blink the inbuilt LED
  pinMode(HEARTBEAT_LED, OUTPUT);
  digitalWrite(HEARTBEAT_LED, LOW);

  m_phaseIndex = 0;
  m_nextTransitionMs = 0;
  m_phaseInitialized = false;

  InitializeTaskScheduler();

  static TsTask heartbeatTask(TASK_IMMEDIATE, TASK_FOREVER, HeartbeatLoop);
  GetTaskScheduler().addTask(heartbeatTask);
  heartbeatTask.enable();
}

void HeartbeatThread::Loop()
{
  const unsigned long now = millis();

  if (!m_phaseInitialized)
  {
    SetPhase(0, now);
    return;
  }

  if (static_cast<long>(now - m_nextTransitionMs) < 0)
    return;

  const unsigned int nextPhase = (m_phaseIndex + 1) % HEARTBEAT_PHASE_COUNT;
  SetPhase(nextPhase, now);
}

void HeartbeatThread::SetPhase(unsigned int phase, unsigned long now)
{
  if (phase >= HEARTBEAT_PHASE_COUNT)
    return;

  const HeartbeatPhase& heartbeatPhase = HEARTBEAT_SEQUENCE[phase];
  digitalWrite(HEARTBEAT_LED, heartbeatPhase.levelHigh ? HIGH : LOW);

  m_phaseIndex = phase;
  m_nextTransitionMs = now + heartbeatPhase.durationMs;
  m_phaseInitialized = true;
}

void HeartbeatThread::HeartbeatLoop()
{
  GetInstance().Loop();
}
