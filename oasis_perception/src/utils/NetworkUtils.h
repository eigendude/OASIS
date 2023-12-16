/*
 *  Copyright (C) 2022-2023 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <string>

namespace OASIS
{
namespace UTILS
{

class NetworkUtils
{
public:
  /*!
   * \brief Get the system hostname
   *
   * \return The hostname, or empty if unknown
   */
  static std::string GetHostName();
};

} // namespace UTILS
} // namespace OASIS
