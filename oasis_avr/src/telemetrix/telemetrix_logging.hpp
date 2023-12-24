/*
 *  Copyright (C) 2023 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#pragma once

namespace OASIS
{
class TelemetrixLogging
{
public:
  static void LogString(const char* stringData);

  static void EnableLogging();
  static void DisableLogging() { m_loggingEnabled = false; }

private:
  static bool m_loggingEnabled;
};
} // namespace OASIS
