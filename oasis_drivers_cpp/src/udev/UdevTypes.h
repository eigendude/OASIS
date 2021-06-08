/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of Oasis - https://github.com/eigendude/oasis
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <stdint.h>
#include <string>

namespace OASIS
{
namespace UDEV
{

enum class UdevDeviceType
{
  DEVICE_TYPE_UNKNOWN,
  DEVICE_TYPE_PER_INTERFACE,
  DEVICE_TYPE_AUDIO,
  DEVICE_TYPE_COMM,
  DEVICE_TYPE_HID,
  DEVICE_TYPE_PHYSICAL,
  DEVICE_TYPE_PTP,
  DEVICE_TYPE_PRINTER,
  DEVICE_TYPE_DISK,
  DEVICE_TYPE_HUB,
  DEVICE_TYPE_DATA,
  DEVICE_TYPE_MISCELLANEOUS,
};

struct UdevDeviceInfo
{
  UdevDeviceType deviceType;
  int udevDeviceType;
  uint16_t vendorId;
  uint16_t productId;
  std::string devicePath;
};

}
}
