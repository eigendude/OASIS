/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "task_scheduler.hpp"

// Pull in the TaskScheduler implementation once so the generated binary links
// against the definitions provided by the Arduino library. Other translation
// units only include the declarations header to avoid duplicate symbol errors.
#include <TScheduler.hpp>

namespace OASIS
{
namespace
{
TsScheduler g_scheduler;
bool g_initialized{false};
} // namespace

void InitializeTaskScheduler()
{
  if (!g_initialized)
  {
    g_scheduler.init();
    g_initialized = true;
  }
}

TsScheduler& GetTaskScheduler()
{
  InitializeTaskScheduler();
  return g_scheduler;
}

void RunTaskScheduler()
{
  InitializeTaskScheduler();
  g_scheduler.execute();
}

} // namespace OASIS
