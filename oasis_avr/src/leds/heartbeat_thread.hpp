/*
 *  Copyright (C) 2021-2024 Garrett Brown
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

  // Static threading functions
  static void HeartbeatLoop();
};

} // namespace OASIS
