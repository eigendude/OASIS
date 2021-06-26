/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <ctime>
#include <libcec/cectypes.h>
#include <string>

namespace OASIS
{
namespace CEC
{

class CecTranslator
{
public:
  /*!
   * \brief Translate a libCEC alert emum to a string suitable for logging
   *
   * \param alert The libCEC alert enum value
   *
   * \return The translated string, or "unknown (<int>)" if unknown
   */
  static std::string TranslateAlert(::CEC::libcec_alert alert);

  /*!
   * \brief Translate a CEC logical address to a string suitable for logging
   *
   * \param address The CEC logical address
   *
   * \return The translated string, or "unknown (<int>)" if unknown
   */
  static std::string TranslateLogicalAddress(::CEC::cec_logical_address address);

  /*!
   * \brief Translate a group of CEC logical addresses to a string suitable for
   * logging
   *
   * \param address The CEC logical addresses
   *
   * \return The string "[<address 1>, <address 2>, ...]" or "[]" if no addresses
   */
  static std::string TranslateLogicalAddresses(const ::CEC::cec_logical_addresses& addresses);

  /*!
   * \brief Translate a CEC opcode emum to a string suitable for logging
   *
   * \param alert The CEC opcode enum value
   *
   * \return The translated string, or "unknown (<hex>)" if unknown
   */
  static std::string TranslateOpcode(::CEC::cec_opcode opcode);

  /*!
   * \brief Translate a CEC keycode emum to a string suitable for logging
   *
   * \param alert The CEC keycode enum value
   *
   * \return The translated string, or "unknown (<hex>)" if unknown
   */
  static std::string TranslateKeyCode(::CEC::cec_user_control_code keycode);

  /*!
   * \brief Translate a CEC vendor ID to the name of the vendor
   *
   * \param vendorId The 24-bit CEC vendor ID
   *
   * \return The vendor name, or the empty string ("") if unknown
   */
  static std::string TranslateVendorID(::CEC::cec_vendor_id vendorId);

  /*!
   * \brief Translate a firmware build date to its ISO-8601 representation
   *
   * \param buildDate The firmware build date, as a Unix timestamp (time_t)
   *
   * \return The ISO-8601 string representation of the build date
   */
  static std::string TranslateBuildDate(std::time_t buildDate);
};

}
}
