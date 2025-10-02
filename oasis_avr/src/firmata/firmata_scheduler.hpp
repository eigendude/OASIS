/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */
#pragma once

// Use the declarations-only header to avoid instantiating TaskScheduler
// definitions in every translation unit. The actual implementation is
// provided by the TaskScheduler Arduino library that we link against.
#include <TSchedulerDeclarations.hpp>

namespace OASIS
{

// Initialize the shared TaskScheduler instance.
void InitializeTaskScheduler();

// Retrieve the shared TaskScheduler instance.
TsScheduler& GetTaskScheduler();

// Execute one scheduler pass. Call from the Arduino loop().
void RunTaskScheduler();

} // namespace OASIS
