/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "CecServer.h"

#include "cec/CecAdapter.h"
#include "cec/CecUtils.h"
#include "udev/UdevScanner.h"
#include "udev/UdevTranslator.h"
#include "udev/UdevUtils.h"

#include <cstdio>
#include <ctime>
#include <libcec/cec.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <set>
#include <stdint.h>
#include <vector>

using namespace OASIS;
using namespace OASIS::CEC;

CecServer::CecServer(ICecCallback& callback, std::shared_ptr<rclcpp::Node> node, rclcpp::Logger& logger)
  : m_callback(callback),
    m_node(std::move(node)),
    m_logger(logger)
{
}

CecServer::~CecServer()
{
  Deinitialize();
}

bool CecServer::Initialize()
{
  // Initialize CEC
  m_cecAdapter = CECInitialise(&m_configuration);
  if (m_cecAdapter == nullptr)
    return false;

  // Initialize udev
  m_deviceScanner = std::make_unique<UDEV::UdevScanner>(*this, m_logger);
  if (!m_deviceScanner->Initialize())
    return false;

  return true;
}

void CecServer::Deinitialize()
{
  // Deinitialize udev
  if (m_deviceScanner)
  {
    m_deviceScanner->Deinitialize();
    m_deviceScanner.reset();
  }

  // Deinitialize  CEC
  m_adapters.clear();
  if (m_cecAdapter != nullptr)
  {
    CECDestroy(m_cecAdapter);
    m_cecAdapter = nullptr;
  }
}

void CecServer::OnDeviceAdded(const UDEV::UdevDeviceInfo& deviceInfo)
{
  RCLCPP_DEBUG(m_logger, "Device added");
  RCLCPP_DEBUG(m_logger, "  VID: %s, PID: %s",
      UDEV::UdevUtils::UsbIdToHexString(deviceInfo.vendorId).c_str(),
      UDEV::UdevUtils::UsbIdToHexString(deviceInfo.productId).c_str());
  RCLCPP_DEBUG(m_logger, "  Class: %s (%s)",
      UDEV::UdevTranslator::TypeToString(deviceInfo.deviceType),
      UDEV::UdevUtils::UdevDeviceClassToHexString(deviceInfo.udevDeviceType).c_str());
  RCLCPP_DEBUG(m_logger, "  Path:  %s", deviceInfo.devicePath.c_str());
}

void CecServer::OnDeviceRemoved(const UDEV::UdevDeviceInfo& deviceInfo)
{
  RCLCPP_DEBUG(m_logger, "Device removed");
  RCLCPP_DEBUG(m_logger, "  VID: %s, PID: %s",
      UDEV::UdevUtils::UsbIdToHexString(deviceInfo.vendorId).c_str(),
      UDEV::UdevUtils::UsbIdToHexString(deviceInfo.productId).c_str());
  RCLCPP_DEBUG(m_logger, "  Class: %s (%s)",
      UDEV::UdevTranslator::TypeToString(deviceInfo.deviceType),
      UDEV::UdevUtils::UdevDeviceClassToHexString(deviceInfo.udevDeviceType).c_str());
  RCLCPP_DEBUG(m_logger, "  Path:  %s", deviceInfo.devicePath.c_str());
}

void CecServer::OnDevicesChanged()
{
  if (m_cecAdapter == nullptr)
    return;

  // Get list of old and new paths for comparison
  std::vector<std::string> oldPaths;
  std::set<std::string> currentPaths;

  std::transform(m_adapters.begin(), m_adapters.end(), std::back_inserter(oldPaths),
    [](const CecAdapterMap::value_type &pair)
    {
      return pair.first;
    }
  );

  ::CEC::cec_adapter_descriptor deviceList[10]{};
  const int8_t iFound = m_cecAdapter->DetectAdapters(deviceList,
      sizeof(deviceList) / sizeof(*deviceList), nullptr, true);

  for (uint8_t iDevicePtr = 0; iDevicePtr < iFound; iDevicePtr++)
  {
    std::string devicePath = deviceList[iDevicePtr].strComPath;
    std::string deviceNode = deviceList[iDevicePtr].strComName;
    uint16_t vendorId = deviceList[iDevicePtr].iVendorId;
    uint16_t productId = deviceList[iDevicePtr].iProductId;

    // Handle new adapters
    auto it = m_adapters.find(devicePath);
    if (it == m_adapters.end())
    {
      // Log adapter info
      RCLCPP_INFO(m_logger, "CEC adapter connected");
      RCLCPP_INFO(m_logger, "  VID: %s, PID: %s",
          UDEV::UdevUtils::UsbIdToHexString(vendorId).c_str(),
          UDEV::UdevUtils::UsbIdToHexString(productId).c_str());
      RCLCPP_INFO(m_logger, "  Node: %s", deviceNode.c_str());
      RCLCPP_INFO(m_logger, "  Path: %s", devicePath.c_str());

      std::unique_ptr<CecAdapter> adapter = std::make_unique<CecAdapter>(m_callback, devicePath, std::move(deviceNode), m_logger);
      if (adapter->Initialize())
        m_adapters.insert(std::make_pair(std::move(devicePath), std::move(adapter)));
    }

    currentPaths.emplace(std::move(devicePath));
  }

  // Handle removed adapters
  for (const std::string& devicePath : oldPaths)
  {
    if (currentPaths.find(devicePath) == currentPaths.end())
    {
      auto it = m_adapters.find(devicePath);

      // Log adapter info
      RCLCPP_INFO(m_logger, "CEC adapter disconnected");
      RCLCPP_INFO(m_logger, "  Node: %s", it->second->GetDeviceNode().c_str());
      RCLCPP_INFO(m_logger, "  Path: %s", it->second->GetDevicePath().c_str());

      it->second->Deinitialize();
      m_adapters.erase(it);
    }
  }
}

bool CecServer::PowerOnDevice(const std::string& device)
{
  auto it = m_adapters.find(device);
  if (it == m_adapters.end())
  {
    RCLCPP_ERROR(m_logger, "CEC not connected to %s", device.c_str());
    return false;
  }

  return it->second->PowerOnDevice();
}

bool CecServer::StandbyDevice(const std::string& device)
{
  auto it = m_adapters.find(device);
  if (it == m_adapters.end())
  {
    RCLCPP_ERROR(m_logger, "CEC not connected to %s", device.c_str());
    return false;
  }


  return it->second->StandbyDevice();
}
