/*
 *  Copyright (C) 2021-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */
#pragma once

#include <stdint.h>

namespace OASIS
{
class Timer
{
public:
  Timer();

  bool IsExpired();
  void SetTimeout(uint32_t intervalMs);
  void Reset();
  uint32_t TimeLeft() const;

private:
  uint32_t m_startMs;
  uint32_t m_intervalMs;
};
} // namespace OASIS
