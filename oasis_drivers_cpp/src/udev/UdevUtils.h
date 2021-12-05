/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <string>

namespace OASIS
{
namespace UDEV
{

class UdevUtils
{
public:
  /*!
   * \dev Convert a USB vendor ID or product ID to a hex string
   *
   * \param usbId The vendor ID or product ID
   *
   * \return The translated hex string, with a leading "0x"
   */
  static std::string UsbIdToHexString(uint16_t usbId);

  /*!
   * \dev Convert a udev device class to a hex string
   *
   * \param udevDeviceClass The device class reported by udev
   *
   * \return The translated hex string, with a leading "0x"
   */
  static std::string UdevDeviceClassToHexString(int udevDeviceClass);
};

} // namespace UDEV
} // namespace OASIS
