/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <stdint.h>
#include <string>

namespace CEC
{
  struct cec_datapacket;
}

namespace OASIS
{
namespace CEC
{

class CecUtils
{
public:
  /*!
   * \dev Convert an 8-bit integer to a hex string
   *
   * \param value The byte value
   *
   * \return The translated hex string, with a leading "0x"
   */
  static std::string ByteToHexString(uint8_t value);

  /*!
   * \dev Convert a physical address to a hex string
   *
   * \param address The CEC physical address
   *
   * \return The translated hex string, with a leading "0x"
   */
  static std::string PhysicalAdressToHexString(uint16_t address);

  /*!
   * \dev Convert a libCEC parameters struct to string array
   *
   * \param parameters The libCEC parameters struct
   *
   * \return An array of hex bytes, e.g. "[0x00, 0x01, ...]"
   */
  static std::string ParametersToHexArray(const ::CEC::cec_datapacket& parameters);
};

}
}
