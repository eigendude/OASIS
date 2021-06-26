/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "LedServer.h"
#include "ILedBehavior.h"
#include "LedCommand.h"
#include "led/behaviors/BusySignal.h"
#include "led/behaviors/FullBrightness.h"
#include "led/behaviors/ProgressBar.h"
#include "led/threads/LedThread.h"

#include <iostream> // TODO

using namespace OASIS;
using namespace LED;

LedServer::LedServer(LedVector leds)
{
  LedCommandMap behaviors = {
      { LedCommandType::BUSY_SIGNAL, LedBehaviorPtr(new BusySignal(leds)) },
      { LedCommandType::FULL_BRIGHTNESS, LedBehaviorPtr(new FullBrightness(leds)) },
      { LedCommandType::PROGRESS_BAR, LedBehaviorPtr(new ProgressBar(leds)) },
  };

  m_behaviors = std::move(behaviors);
}

LedServer::~LedServer()
{
  Deinitialize();
}

bool LedServer::Initialize()
{
  m_thread.reset(new LedThread(*this));
  m_thread->Initialize();

  return true;
}

void LedServer::Deinitialize()
{
  auto it = m_behaviors.find(m_currentType);
  if (it != m_behaviors.end())
    it->second->Shutdown();
  m_behaviors.clear();

  if (m_thread)
  {
    m_thread->Abort();
    m_thread->Deinitialize();
    m_thread.reset();
  }
}

void LedServer::ProcessCommand(const std::string &command)
{
  LedCommand ledCommand(command);
  std::cout << "LedServer: Command " << static_cast<int>(ledCommand.Type()) <<
      ", progress: " << ledCommand.Progress() << std::endl;

  m_currentType = ledCommand.Type();
  m_progressPercent = ledCommand.Progress();
}

void LedServer::Update(int64_t runtimeMs)
{
  auto it = m_behaviors.find(m_currentType);
  if (it != m_behaviors.end())
    it->second->Update(runtimeMs, m_progressPercent / 100.0f);
}
