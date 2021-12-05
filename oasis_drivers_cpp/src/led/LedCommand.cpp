/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "LedCommand.h"

#include <sstream>

using namespace OASIS;
using namespace LED;

LedCommand::LedCommand(const std::string& line)
{
  ParseLine(line);
}

void LedCommand::ParseLine(const std::string& line)
{
  std::istringstream lineStr(line);

  std::string command;
  lineStr >> command;
  m_type = TranslateType(command);

  lineStr >> m_progressPercent;
}

LedCommandType LedCommand::TranslateType(const std::string& commandType)
{
  if (commandType == "busy")
    return LedCommandType::BUSY_SIGNAL;
  else if (commandType == "full")
    return LedCommandType::FULL_BRIGHTNESS;
  else if (commandType == "progress")
    return LedCommandType::PROGRESS_BAR;

  return LedCommandType::FULL_BRIGHTNESS;
}
