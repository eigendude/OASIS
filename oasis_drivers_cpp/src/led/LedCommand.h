/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */
#pragma once

#include "LedTypes.h"

#include <string>

namespace OASIS
{
namespace LED
{

class LedCommand
{
public:
  /*!
   * \brief Construct an LED command from a line of text
   */
  LedCommand(const std::string& line);

  /*!
   * \brief Get the type of LED command
   */
  LedCommandType Type() const { return m_type; }

  /*!
   * \brief Get the progress of this task, in percent, if known
   *
   * \return The progress percent between 0 and 100, or 0 if unknown
   */
  unsigned int Progress() const { return m_progressPercent; }

private:
  // Utility functions
  void ParseLine(const std::string& line);
  static LedCommandType TranslateType(const std::string& commandType);

  // Command parameters
  LedCommandType m_type = LedCommandType::FULL_BRIGHTNESS;
  unsigned int m_progressPercent = 0;
};

} // namespace LED
} // namespace OASIS
