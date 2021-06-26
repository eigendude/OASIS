/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "UdevUtils.h"

#include "utils/StringUtils.h"

#include <limits>

using namespace OASIS;
using namespace UDEV;

std::string UdevUtils::UsbIdToHexString(uint16_t usbId)
{
  return UTILS::StringUtils::ToHexString(usbId);
}

std::string UdevUtils::UdevDeviceClassToHexString(int udevDeviceClass)
{
  if (0 <= udevDeviceClass && udevDeviceClass <= std::numeric_limits<uint8_t>::max())
    return UTILS::StringUtils::ToHexString(static_cast<uint8_t>(udevDeviceClass));

  if (0 <= udevDeviceClass && udevDeviceClass <= std::numeric_limits<uint16_t>::max())
    return UTILS::StringUtils::ToHexString(static_cast<uint16_t>(udevDeviceClass));

  return UTILS::StringUtils::ToHexString(static_cast<uint32_t>(udevDeviceClass));
}
