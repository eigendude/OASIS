/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */
#pragma once

#include <memory>

namespace oasis_perception
{
class HelloWorld
{
public:
  HelloWorld();
  ~HelloWorld();

  void Initialize();
  void Run();

private:
  bool PrintHelloWorld();
};
} // namespace oasis_perception
