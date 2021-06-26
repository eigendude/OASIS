/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "UdevTypes.h"

namespace OASIS
{
namespace UDEV
{

class UdevTranslator
{
public:
  /*!
   * \def Translate a udev device class ID to an enum
   *
   * \param deviceClass The Device and/or Interface Class code as defined by
   * www.usb.org documents. See "UdevDefinitions.h".
   */
  static UdevDeviceType GetDeviceType(int deviceClass);

  /*!
   * \def Translate a device type enum to a string
   *
   * \param deviceType The device type enum
   *
   * \return A string representation of the type suitable for logging, or
   * "unknown" if the type is unknown
   */
  static const char *TypeToString(UdevDeviceType deviceType);
};

}
}
