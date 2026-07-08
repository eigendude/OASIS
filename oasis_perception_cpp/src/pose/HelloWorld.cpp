/*
 *  Copyright (C) 2025-2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "HelloWorld.h"

using namespace oasis_perception;

HelloWorld::HelloWorld() = default;

HelloWorld::~HelloWorld() = default;

void HelloWorld::Initialize()
{
}

mediapipe_facade::HelloWorldResult HelloWorld::Run()
{
  return mediapipe_facade::RunHelloWorld();
}
