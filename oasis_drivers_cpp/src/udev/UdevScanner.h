/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "UdevTypes.h"

#include <atomic>
#include <map>
#include <memory>
#include <string>
#include <thread>

struct udev;
struct udev_monitor;

namespace rclcpp
{
class Logger;
}

namespace OASIS
{
namespace UDEV
{

class IUdevCallback;

class UdevScanner
{
public:
  UdevScanner(IUdevCallback& callback, rclcpp::Logger& logger);
  ~UdevScanner();

  // Lifecycle functions
  bool Initialize();
  void Deinitialize();

private:
  // Type aliases
  using UdevDeviceMap = std::map<std::string, UdevDeviceInfo>;

  // Thread entry point
  void Process();

  // udev functions
  bool ScanForDevices();
  bool WaitForUpdate();
  void HandleScanResults(const UdevDeviceMap& oldDevices, const UdevDeviceMap& newDevices);

  // Construction parameters
  IUdevCallback& m_callback;
  rclcpp::Logger& m_logger;

  // udev parameters
  struct udev* m_udev = nullptr;
  struct udev_monitor* m_udevMon = nullptr;
  UdevDeviceMap m_devices;

  // Threading parameters
  std::unique_ptr<std::thread> m_thread;
  std::atomic<bool> m_bStop = false;
};

} // namespace UDEV
} // namespace OASIS
