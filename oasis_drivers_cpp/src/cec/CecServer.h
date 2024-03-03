/*
 *  Copyright (C) 2021-2024 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "udev/IUdevCallback.h"

#include <map>
#include <memory>
#include <string>

#include <libcec/cectypes.h>

namespace CEC
{
class ICECAdapter;
}

namespace rclcpp
{
class Logger;
class Node;
} // namespace rclcpp

namespace OASIS
{
namespace UDEV
{
class UdevScanner;
}

namespace CEC
{

class CecAdapter;
class ICecCallback;

class CecServer : public UDEV::IUdevCallback
{
public:
  CecServer(ICecCallback& callback, std::shared_ptr<rclcpp::Node> node, rclcpp::Logger& logger);
  ~CecServer() override;

  // Lifecycle functions
  bool Initialize();
  void Deinitialize();

  // Implementation of IUdevCallback
  void OnDeviceAdded(const UDEV::UdevDeviceInfo& deviceInfo) override;
  void OnDeviceRemoved(const UDEV::UdevDeviceInfo& deviceInfo) override;
  void OnDevicesChanged() override;

  // CEC interface
  bool PowerOnDevice(const std::string& device);
  bool StandbyDevice(const std::string& device);

private:
  // Type aliases
  using CecAdapterMap = std::map<std::string, std::unique_ptr<CecAdapter>>; // Path -> adapter

  // Construction parameters
  ICecCallback& m_callback;
  const std::shared_ptr<rclcpp::Node> m_node;
  rclcpp::Logger& m_logger;

  // udev parameters
  std::unique_ptr<UDEV::UdevScanner> m_deviceScanner;

  // CEC parameters
  CecAdapterMap m_adapters;
  ::CEC::ICECAdapter* m_cecAdapter = nullptr;
  ::CEC::libcec_configuration m_configuration{};
};

} // namespace CEC
} // namespace OASIS
