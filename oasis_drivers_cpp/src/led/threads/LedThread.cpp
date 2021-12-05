/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "LedThread.h"

#include "LedThreadCondition.h"
#include "led/LedServer.h"

using namespace OASIS;
using namespace LED;

namespace
{
constexpr unsigned int PWM_UPDATE_HZ = 4;
}

LedThread::LedThread(LedServer& server) : m_server(server)
{
}

LedThread::~LedThread() = default;

void LedThread::Initialize()
{
  m_condition.reset(new LedThreadCondition);
  m_thread.reset(new std::thread(&LedThread::Process, this));
}

void LedThread::Deinitialize()
{
  if (m_thread)
  {
    m_thread->join();
    m_thread.reset();
  }
  m_condition.reset();
}

void LedThread::Process()
{
  // Verify initialization
  if (!m_condition)
    return;

  m_elapsedMs = 0;

  while (true)
  {
    RunOnce();

    // TODO: Improve timing
    const int64_t timeoutMs = 1000 / PWM_UPDATE_HZ;
    if (!m_condition->Wait(timeoutMs))
      break;

    m_elapsedMs += timeoutMs;
  }
}

void LedThread::Abort()
{
  m_condition->Notify();
}

void LedThread::RunOnce()
{
  m_server.Update(m_elapsedMs);
}
