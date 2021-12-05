/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "UdevScanner.h"

#include "IUdevCallback.h"
#include "UdevDefinitions.h"
#include "UdevTranslator.h"
#include "utils/StringUtils.h"

#include <algorithm>
#include <errno.h>
#include <utility>
#include <vector>

#include <poll.h>
#include <rclcpp/logging.hpp>

extern "C"
{
#include <libudev.h>
}

using namespace OASIS;
using namespace UDEV;

UdevScanner::UdevScanner(IUdevCallback& callback, rclcpp::Logger& logger)
  : m_callback(callback), m_logger(logger)
{
}

UdevScanner::~UdevScanner()
{
  Deinitialize();
}

bool UdevScanner::Initialize()
{
  m_udev = udev_new();
  if (m_udev == nullptr)
  {
    RCLCPP_ERROR(m_logger, "Failed to allocate udev context");
    return false;
  }

  // Set up a devices monitor that listen for any device change
  m_udevMon = udev_monitor_new_from_netlink(m_udev, "udev");
  if (m_udevMon == nullptr)
  {
    RCLCPP_ERROR(m_logger, "Failed to allocate udev monitor");
    return false;
  }

  // Filter to only receive USB events
  if (udev_monitor_filter_add_match_subsystem_devtype(m_udevMon, "usb", nullptr) < 0)
    RCLCPP_ERROR(m_logger, "Could not limit filter on USB only");

  int result = udev_monitor_enable_receiving(m_udevMon);
  if (result < 0)
    RCLCPP_ERROR(m_logger, "Monitor failed to enable receiving (%d)", result);

  m_bStop = false;
  m_thread = std::make_unique<std::thread>(&UdevScanner::Process, this);

  return true;
}

void UdevScanner::Deinitialize()
{
  m_bStop = true;
  if (m_thread)
  {
    m_thread->join();
    m_thread.reset();
  }

  if (m_udevMon != nullptr)
  {
    udev_monitor_unref(m_udevMon);
    m_udevMon = nullptr;
  }

  if (m_udev != nullptr)
  {
    udev_unref(m_udev);
    m_udev = nullptr;
  }
}

void UdevScanner::Process()
{
  if (!ScanForDevices())
    return;

  while (!m_bStop)
  {
    bool bUpdated = WaitForUpdate();
    if (bUpdated && !m_bStop)
    {
      if (!ScanForDevices())
        break;
    }
  }
}

bool UdevScanner::ScanForDevices()
{
  std::map<std::string, UdevDeviceInfo> newDevices;

  struct udev_enumerate* enumerate = udev_enumerate_new(m_udev);
  if (enumerate == nullptr)
  {
    RCLCPP_ERROR(m_logger, "Failed to enumerate udev devices");
    return false;
  }

  int result = udev_enumerate_scan_devices(enumerate);
  if (result < 0)
  {
    RCLCPP_ERROR(m_logger, "Failed to scan udev devices");
    return false;
  }

  struct udev_list_entry* devices = udev_enumerate_get_list_entry(enumerate);
  if (devices == nullptr)
  {
    RCLCPP_ERROR(m_logger, "Failed to list udev devices");
    return false;
  }

  struct udev_list_entry* dev_list_entry = nullptr;
  udev_list_entry_foreach(dev_list_entry, devices)
  {
    bool bContinue = true;
    struct udev_device* parent = nullptr;
    struct udev_device* dev = nullptr;
    std::string strClass;

    std::string strPath = udev_list_entry_get_name(dev_list_entry);
    if (strPath.empty())
      bContinue = false;
    ;

    if (bContinue)
    {
      parent = udev_device_new_from_syspath(m_udev, strPath.c_str());
      if (parent == nullptr)
        bContinue = false;
      ;
    }

    if (bContinue)
    {
      dev = udev_device_get_parent(udev_device_get_parent(parent));
      if (dev == nullptr || !udev_device_get_sysattr_value(dev, "idVendor") ||
          !udev_device_get_sysattr_value(dev, "idProduct"))
        bContinue = false;
    }

    if (bContinue)
    {
      strClass = udev_device_get_sysattr_value(dev, "bDeviceClass");
      if (strClass.empty())
        bContinue = false;
    }

    if (bContinue)
    {
      // Translate parameters
      //std::string devicePath = udev_device_get_syspath(dev);
      std::string devicePath = udev_device_get_syspath(dev);
      if (newDevices.find(devicePath) == newDevices.end())
      {
        int iClass = UTILS::StringUtils::HexStringToInt(strClass);
        UdevDeviceType deviceType = UdevTranslator::GetDeviceType(iClass);
        uint16_t vendorId = static_cast<uint16_t>(
            UTILS::StringUtils::HexStringToInt(udev_device_get_sysattr_value(dev, "idVendor")));
        uint16_t productId = static_cast<uint16_t>(
            UTILS::StringUtils::HexStringToInt(udev_device_get_sysattr_value(dev, "idProduct")));

        // Record device
        UdevDeviceInfo deviceInfo{deviceType, iClass, vendorId, productId, devicePath};
        newDevices.insert({std::move(devicePath), std::move(deviceInfo)});
      }
    }

    if (parent != nullptr)
    {
      // Unref the parent device
      udev_device_unref(parent);
    }
  }

  // Free the enumerator object
  udev_enumerate_unref(enumerate);

  // Compare new devices and old devices and invoke callbacks
  HandleScanResults(m_devices, newDevices);

  // Record the new devices
  m_devices = std::move(newDevices);

  return true;
}

bool UdevScanner::WaitForUpdate()
{
  const int udevFd = udev_monitor_get_fd(m_udevMon);
  if (udevFd < 0)
  {
    RCLCPP_ERROR(m_logger, "Failed to get udev monitor file descriptor");
    return false;
  }

  // Poll for udev changes
  struct pollfd pollFd;
  pollFd.fd = udevFd;
  pollFd.events = POLLIN;

  int iPollResult;
  while (!m_bStop && (iPollResult = poll(&pollFd, 1, 100)) <= 0)
  {
    if (errno != EINTR && iPollResult != 0)
      break;
  }

  // If thread is being stopped, just return false
  if (m_bStop)
    return false;

  // We have to read the message from the queue, even though we're not
  // actually using it
  struct udev_device* dev = udev_monitor_receive_device(m_udevMon);
  if (dev != nullptr)
    udev_device_unref(dev);
  else
  {
    RCLCPP_ERROR(m_logger, "Failed to get device from udev_monitor_receive_device()");
    return false;
  }

  return true;
}

void UdevScanner::HandleScanResults(const UdevDeviceMap& oldDevices,
                                    const UdevDeviceMap& newDevices)
{
  using UdevDeviceMapPair = std::pair<std::string, UdevDeviceInfo>;

  std::vector<UdevDeviceMapPair> added;
  std::vector<UdevDeviceMapPair> removed;

  auto cmp = [](const UdevDeviceMapPair& a, const UdevDeviceMapPair& b)
  { return a.first < b.first; };

  std::set_difference(newDevices.begin(), newDevices.end(), oldDevices.begin(), oldDevices.end(),
                      std::back_inserter(added), cmp);
  std::set_difference(oldDevices.begin(), oldDevices.end(), newDevices.begin(), newDevices.end(),
                      std::back_inserter(removed), cmp);

  for (const UdevDeviceMapPair& addedDevice : added)
    m_callback.OnDeviceAdded(addedDevice.second);

  for (const UdevDeviceMapPair& removedDevice : removed)
    m_callback.OnDeviceRemoved(removedDevice.second);

  const bool bChanged = !added.empty() || !removed.empty();
  if (bChanged)
    m_callback.OnDevicesChanged();
}
