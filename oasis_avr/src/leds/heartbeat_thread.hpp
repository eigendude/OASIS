/*
 *  Copyright (C) 2021-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */
#pragma once

namespace OASIS
{

class HeartbeatThread
{
public:
  static HeartbeatThread& GetInstance();

  void Setup();

private:
  // Threading functions
  void Loop();

  void SetPhase(unsigned int phase, unsigned long now);

  // Static threading functions
  static void HeartbeatLoop();

  // State parameters
  unsigned int m_phaseIndex = 0;
  unsigned long m_nextTransitionMs = 0;
  bool m_phaseInitialized = false;
};

} // namespace OASIS
