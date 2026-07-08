/*
 *  Copyright (C) 2025-2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */
#pragma once

#include "MediaPipeFacade.h"

namespace oasis_perception
{
class HelloWorld
{
public:
  HelloWorld();
  ~HelloWorld();

  void Initialize();
  mediapipe_facade::HelloWorldResult Run();
};
} // namespace oasis_perception
