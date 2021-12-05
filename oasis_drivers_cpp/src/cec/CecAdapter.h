/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "ICecAdapter.h"

#include <memory>
#include <mutex>
#include <string>

#include <libcec/cectypes.h>

namespace CEC
{
class ICECAdapter;
struct ICECCallbacks;
struct libcec_configuration;
} // namespace CEC

namespace rclcpp
{
class Logger;
}

namespace OASIS
{
namespace CEC
{

class ICecCallback;

class CecAdapter : public ICecAdapter
{
public:
  CecAdapter(ICecCallback& callback,
             std::string devicePath,
             std::string deviceNode,
             rclcpp::Logger& logger);
  ~CecAdapter();

  // Lifecycle functions
  bool Initialize();
  void Deinitialize();

  // Implementation of ICecAdapter
  const std::string& GetDevicePath() const override { return m_devicePath; }
  const std::string& GetDeviceNode() const override { return m_deviceNode; }

  // CEC interface
  bool PowerOnDevice();
  bool StandbyDevice();

private:
  // Lifecycle functions
  void InitializeConfiguration();
  bool OpenAdapter();

  // CEC functions
  void SetConfigurationFromLibCEC(const ::CEC::libcec_configuration& m_configuration);
  void SetVendorFromCEC(const std::string& vendorName);
  void OnPowerOn();
  void OnStandby();
  void ReopenConnection(bool bAsync);

  // Internal callbacks from libCEC
  void CecLogMessage(const ::CEC::cec_log_message* message);
  void CecKeyPress(const ::CEC::cec_keypress* key);
  void CecCommand(const ::CEC::cec_command* command);
  void CecConfiguration(const ::CEC::libcec_configuration* config);
  void CecAlert(const ::CEC::libcec_alert alert, const ::CEC::libcec_parameter data);
  int CecMenuStateChanged(const ::CEC::cec_menu_state state);
  void CecSourceActivated(const ::CEC::cec_logical_address address, const uint8_t activated);

  // External callbacks from libCEC
  static void CecLogMessage(void* cbParam, const ::CEC::cec_log_message* message);
  static void CecKeyPress(void* cbParam, const ::CEC::cec_keypress* key);
  static void CecCommand(void* cbParam, const ::CEC::cec_command* command);
  static void CecConfiguration(void* cbParam, const ::CEC::libcec_configuration* config);
  static void CecAlert(void* cbParam,
                       const ::CEC::libcec_alert alert,
                       const ::CEC::libcec_parameter data);
  static int CecMenuStateChanged(void* cbParam, const ::CEC::cec_menu_state state);
  static void CecSourceActivated(void* cbParam,
                                 const ::CEC::cec_logical_address address,
                                 const uint8_t activated);

  // Construction parameters
  ICecCallback& m_callback;
  const std::string m_devicePath;
  const std::string m_deviceNode;
  rclcpp::Logger& m_logger;

  // CEC parameters
  ::CEC::ICECAdapter* m_cecAdapter = nullptr;
  ::CEC::libcec_configuration m_configuration{};
  ::CEC::ICECCallbacks m_callbacks{};
  std::string m_vendorName;

  // Threading parameters
  std::mutex m_mutex;
};

} // namespace CEC
} // namespace OASIS
