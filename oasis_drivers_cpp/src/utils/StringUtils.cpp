/*
 *  Copyright (C) 2021-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "StringUtils.h"

#include <cstdint>
#include <cstdio>
#include <iomanip>
#include <sstream>
#include <type_traits>

using namespace OASIS;
using namespace UTILS;

unsigned int StringUtils::HexStringToInt(const std::string& strHex)
{
  unsigned int iVal = 0;

  std::sscanf(strHex.c_str(), "%x", &iVal);

  return iVal;
}

template<typename ValueType>
std::string StringUtils::ToHexString(ValueType value)
{
  // Ensure this function is called with a template parameter that makes sense
  static_assert(
      std::is_integral<ValueType>::value,
      "Template argument 'ValueType' must be a fundamental integer type (e.g. int, short, etc..).");

  std::ostringstream stream;

  // Handle 8-bit integers properly.
  // See https://stackoverflow.com/questions/5100718/integer-to-hex-string-in-c
  stream << "0x" << std::setfill('0') << std::setw(sizeof(ValueType) * 2) << std::hex;

  // If ValueType is an 8-bit integer type (e.g. uint8_t or int8_t) it will be
  // treated as an ASCII code, giving the wrong result. So we use C++17's
  // "if constexpr" to have the compiler decides at compile-time if it's
  // converting an 8-bit int or not.
  if constexpr (std::is_same_v<std::uint8_t, ValueType>)
  {
    // Unsigned 8-bit unsigned int type. Cast to int to avoid ASCII code
    // interpretation of the int. The number of hex digits in the returned
    // string will still be two, which is correct for 8 bits, because of the
    // 'sizeof(ValueType)' above.
    stream << static_cast<int>(value);
  }
  else if (std::is_same_v<std::int8_t, ValueType>)
  {
    // For 8-bit signed int, same as above, except we must first cast to
    // unsigned int, because values above 127d (0x7f) in the int will cause
    // further issues if we cast directly to int.
    stream << static_cast<int>(static_cast<std::uint8_t>(value));
  }
  else
  {
    // No cast needed for ints wider than 8 bits.
    stream << value;
  }

  return stream.str();
}

// Avoid link errors with template functions
// See https://isocpp.org/wiki/faq/templates#separate-template-class-defn-from-decl
template std::string StringUtils::ToHexString<uint8_t>(uint8_t value);
template std::string StringUtils::ToHexString<uint16_t>(uint16_t value);
template std::string StringUtils::ToHexString<unsigned int>(uint32_t value);
