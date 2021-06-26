/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

namespace OASIS
{
namespace UDEV
{

struct UdevDeviceInfo;

class IUdevCallback
{
public:
  virtual ~IUdevCallback() = default;

  /*!
   * \brief Called when a device is added
   *
   * \param deviceInfo The udev info of the device
   */
  virtual void OnDeviceAdded(const UdevDeviceInfo& deviceInfo) = 0;

  /*!
   * \brief Called when a device is removed
   *
   * \param deviceInfo The udev info of the device
   */
  virtual void OnDeviceRemoved(const UdevDeviceInfo& deviceInfo) = 0;

  /*!
   * \brief Called when devices are added or removed
   */
  virtual void OnDevicesChanged() = 0;
};

}
}
