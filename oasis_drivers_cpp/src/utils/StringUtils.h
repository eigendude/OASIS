/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of Oasis - https://github.com/eigendude/oasis
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

class StringUtils
{
public:
  /*!
   * \dev Convert a hex string to an integer
   *
   * \param strHex The hex string
   *
   * \return The translated int, or 0 if invalid hex string
   */
  static unsigned int HexStringToInt(const std::string &strHex);

  /*!
   * \dev Convert an integral value to a hex string
   *
   * \param value The integral value, type must pass std::is_integral
   *
   * \return The hex string
   */
  template<typename ValueType>
  static std::string ToHexString(ValueType value);
};

}
}
