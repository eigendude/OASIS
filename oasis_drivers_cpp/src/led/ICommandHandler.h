/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */
#pragma once

#include <string>

namespace OASIS
{
namespace LED
{

/*!
 * \brief Interface for handling simple commands
 *
 * Relatively basic. Commands arrive in the form of simple strings.
 */
class ICommandHandler
{
public:
  virtual ~ICommandHandler() = default;

  /*!
   * \brief Process command
   *
   * \param command An opaque string of tokens representing a command
   */
  virtual void ProcessCommand(const std::string& command) = 0;
};

} // namespace LED
} // namespace OASIS
