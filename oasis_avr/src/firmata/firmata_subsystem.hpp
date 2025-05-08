/*
 *  Copyright (C) 2022-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */
#pragma once

namespace OASIS
{

class FirmataSubsystem
{
public:
  virtual ~FirmataSubsystem() = default;

  // Lifecycle functions
  virtual void Setup() {}
  virtual void Loop() {}
  virtual void Sample() {}
};

} // namespace OASIS
