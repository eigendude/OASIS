/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of Oasis - https://github.com/eigendude/oasis
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "UdevTranslator.h"

#include "UdevDefinitions.h"

using namespace OASIS;
using namespace UDEV;

UdevDeviceType UdevTranslator::GetDeviceType(int iDeviceClass)
{
  switch (iDeviceClass)
  {
  case USB_CLASS_PER_INTERFACE:
    return UdevDeviceType::DEVICE_TYPE_PER_INTERFACE;
  case USB_CLASS_AUDIO:
    return UdevDeviceType::DEVICE_TYPE_AUDIO;
  case USB_CLASS_COMM:
    return UdevDeviceType::DEVICE_TYPE_COMM;
  case USB_CLASS_HID:
    return UdevDeviceType::DEVICE_TYPE_HID;
  case USB_CLASS_PHYSICAL:
    return UdevDeviceType::DEVICE_TYPE_PHYSICAL;
  case USB_CLASS_PTP:
    return UdevDeviceType::DEVICE_TYPE_PTP;
  case USB_CLASS_PRINTER:
    return UdevDeviceType::DEVICE_TYPE_PRINTER;
  case USB_CLASS_MASS_STORAGE:
    return UdevDeviceType::DEVICE_TYPE_DISK;
  case USB_CLASS_HUB:
    return UdevDeviceType::DEVICE_TYPE_HUB;
  case USB_CLASS_DATA:
    return UdevDeviceType::DEVICE_TYPE_DATA;
  case USB_CLASS_MISCELLANEOUS:
    return UdevDeviceType::DEVICE_TYPE_MISCELLANEOUS;
  default:
    break;
  }

  return UdevDeviceType::DEVICE_TYPE_UNKNOWN;
}

const char *UdevTranslator::TypeToString(UdevDeviceType deviceType)
{
  switch (deviceType)
  {
  case UdevDeviceType::DEVICE_TYPE_PER_INTERFACE:
    return "per interface";
  case UdevDeviceType::DEVICE_TYPE_AUDIO:
    return "audio";
  case UdevDeviceType::DEVICE_TYPE_COMM:
    return "comm";
  case UdevDeviceType::DEVICE_TYPE_HID:
    return "hid";
  case UdevDeviceType::DEVICE_TYPE_PHYSICAL:
    return "physical";
  case UdevDeviceType::DEVICE_TYPE_PTP:
    return "ptp";
  case UdevDeviceType::DEVICE_TYPE_PRINTER:
    return "printer";
  case UdevDeviceType::DEVICE_TYPE_DISK:
    return "disk";
  case UdevDeviceType::DEVICE_TYPE_HUB:
    return "hub";
  case UdevDeviceType::DEVICE_TYPE_DATA:
    return "data";
  case UdevDeviceType::DEVICE_TYPE_MISCELLANEOUS:
    return "miscellaneous";
  default:
    break;
  }

  return "UNKNOWN";
}
