/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "LedThreadCondition.h"

#include <chrono>
#include <thread>

using namespace OASIS;
using namespace LED;

LedThreadCondition::LedThreadCondition() = default;

bool LedThreadCondition::Wait(int64_t timeoutMs)
{
  if (m_valueSet)
    return false;

  std::this_thread::sleep_for(std::chrono::milliseconds(timeoutMs));

  return !m_valueSet;
}

void LedThreadCondition::Notify()
{
  m_valueSet = true;
};
