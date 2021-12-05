/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "CecAdapter.h"

#include "CecTranslator.h"
#include "CecUtils.h"
#include "ICecCallback.h"

#include <cstdio>
#include <cstring>
#include <ctime>

#include <libcec/cec.h>
#include <rclcpp/logging.hpp>

#define CEC_LIB_SUPPORTED_VERSION LIBCEC_VERSION_TO_UINT(4, 0, 0)

using namespace OASIS;
using namespace OASIS::CEC;

namespace
{
constexpr const char* CEC_DEVICE_NAME = "OASIS";
}

CecAdapter::CecAdapter(ICecCallback& callback,
                       std::string devicePath,
                       std::string deviceNode,
                       rclcpp::Logger& logger)
  : m_callback(callback),
    m_devicePath(std::move(devicePath)),
    m_deviceNode(std::move(deviceNode)),
    m_logger(logger)
{
}

CecAdapter::~CecAdapter() = default;

bool CecAdapter::Initialize()
{
  // Set configuration
  InitializeConfiguration();

  // Initialize adapter
  m_cecAdapter = CECInitialise(&m_configuration);
  if (m_cecAdapter == nullptr)
  {
    RCLCPP_ERROR(m_logger, "[%s] Failed to initialize CEC library", m_deviceNode.c_str());
    return false;
  }

  // Log libCEC version
  RCLCPP_INFO(m_logger, "[%s] Using libCEC server v%s", m_deviceNode.c_str(),
              m_cecAdapter->VersionToString(m_configuration.serverVersion).c_str());

  // Open adapter
  return OpenAdapter();
}

void CecAdapter::InitializeConfiguration()
{
  m_configuration.clientVersion = ::CEC::LIBCEC_VERSION_CURRENT;

  // Device name
  std::snprintf(m_configuration.strDeviceName,
                sizeof(m_configuration.strDeviceName) / sizeof(*m_configuration.strDeviceName),
                "%s", CEC_DEVICE_NAME);

  // Set the primary device type
  m_configuration.deviceTypes.Clear();
  m_configuration.deviceTypes.Add(::CEC::CEC_DEVICE_TYPE_TV);

  // Always try to autodetect the address. When the firmware supports this, it
  // will override the physical address, connected device and HDMI port settings
  m_configuration.bAutodetectAddress = CEC_DEFAULT_SETTING_AUTODETECT_ADDRESS;

  // Set the physical address. When set, it will override the connected device
  // and HDMI port settings
  m_configuration.iPhysicalAddress = CEC_PHYSICAL_ADDRESS_TV;

  // Set the connected device
  m_configuration.baseDevice = ::CEC::CECDEVICE_TV;

  /* TODO
  m_configuration.bActivateSource = 0;
  m_configuration.bPowerOffOnStandby = 0;
  m_configuration.bAutoPowerOn = 0;
  */

  // Set callbacks
  m_callbacks.Clear();
  m_callbacks.logMessage = &CecLogMessage;
  m_callbacks.keyPress = &CecKeyPress;
  m_callbacks.commandReceived = &CecCommand;
  m_callbacks.configurationChanged = &CecConfiguration;
  m_callbacks.alert = &CecAlert;
  m_callbacks.menuStateChanged = &CecMenuStateChanged;
  m_callbacks.sourceActivated = &CecSourceActivated;
  m_configuration.callbackParam = this;
  m_configuration.callbacks = &m_callbacks;
}

bool CecAdapter::OpenAdapter()
{
  bool bIsOpen = false;

  RCLCPP_INFO(m_logger, "[%s] Connecting to CEC adapter", m_deviceNode.c_str());

  bIsOpen = m_cecAdapter->Open(m_deviceNode.c_str(), 10000);
  if (!bIsOpen)
  {
    // Display warning: couldn't initialize libCEC
    RCLCPP_ERROR(m_logger, "[%s] Could not opening a connection to the CEC adapter",
                 m_deviceNode.c_str());
  }

  if (bIsOpen)
  {
    std::lock_guard<std::mutex> lock(m_mutex);

    RCLCPP_INFO(m_logger, "[%s] Connection to the CEC adapter opened", m_deviceNode.c_str());

    // Read the configuration
    ::CEC::libcec_configuration config;
    if (m_cecAdapter->GetCurrentConfiguration(&config))
    {
      // Update the local configuration
      SetConfigurationFromLibCEC(config);
    }
    else
    {
      RCLCPP_ERROR(m_logger, "[%s] Failed to read current configuration from CEC adapter",
                   m_deviceNode.c_str());
    }
  }

  return bIsOpen;
}

void CecAdapter::Deinitialize()
{
  if (m_cecAdapter != nullptr)
  {
    // Close adapter
    m_cecAdapter->Close();

    CECDestroy(m_cecAdapter);
    m_cecAdapter = nullptr;
  }
}

// CEC interface
bool CecAdapter::PowerOnDevice()
{
  if (m_cecAdapter == nullptr)
  {
    RCLCPP_ERROR(m_logger, "[%s] CEC adapter not initialized", m_deviceNode.c_str());
    return false;
  }

  if (!m_cecAdapter->PowerOnDevices(::CEC::CECDEVICE_TV))
  {
    RCLCPP_ERROR(m_logger, "[%s] Failed to power on TV", m_deviceNode.c_str());
    return false;
  }

  return true;
}

bool CecAdapter::StandbyDevice()
{
  if (m_cecAdapter == nullptr)
  {
    RCLCPP_ERROR(m_logger, "[%s] CEC adapter not initialized", m_deviceNode.c_str());
    return false;
  }

  if (m_cecAdapter->StandbyDevices(::CEC::CECDEVICE_TV))
  {
    RCLCPP_ERROR(m_logger, "[%s] Failed to put TV in standby", m_deviceNode.c_str());
    return false;
  }

  return true;
}

void CecAdapter::SetConfigurationFromLibCEC(const ::CEC::libcec_configuration& config)
{
  RCLCPP_DEBUG(m_logger, "[%s] Setting configuration from libCEC", m_deviceNode.c_str());

  // Set the primary device type
  m_configuration.deviceTypes.Clear();
  m_configuration.deviceTypes.Add(config.deviceTypes[0]);

  // Set the connected device
  m_configuration.baseDevice = config.baseDevice;
  RCLCPP_DEBUG(
      m_logger, "[%s]   Connected_device: %s", m_deviceNode.c_str(),
      (config.baseDevice == ::CEC::CECDEVICE_AUDIOSYSTEM ? "Amplifier / AVR device" : "TV"));

  // Set the HDMI port number
  m_configuration.iHDMIPort = config.iHDMIPort;
  RCLCPP_DEBUG(m_logger, "[%s]   CEC HDMI port: %d", m_deviceNode.c_str(),
               static_cast<int>(config.iHDMIPort));

  RCLCPP_DEBUG(m_logger, "[%s]   Physical address: %s", m_deviceNode.c_str(),
               CecUtils::PhysicalAdressToHexString(config.iPhysicalAddress).c_str());

  // set the devices to wake when starting
  m_configuration.wakeDevices = config.wakeDevices;
  RCLCPP_DEBUG(m_logger, "[%s]   Wake devices: %s", m_deviceNode.c_str(),
               CecTranslator::TranslateLogicalAddresses(m_configuration.wakeDevices).c_str());

  m_configuration.powerOffDevices = config.powerOffDevices;
  RCLCPP_DEBUG(m_logger, "[%s]   Power off devices: %s", m_deviceNode.c_str(),
               CecTranslator::TranslateLogicalAddresses(m_configuration.powerOffDevices).c_str());

  // Set the boolean settings
  m_configuration.bActivateSource = config.bActivateSource;
  RCLCPP_DEBUG(m_logger, "[%s]   Activate source: %s", m_deviceNode.c_str(),
               (m_configuration.bActivateSource == 1 ? "true" : "false"));

  m_configuration.iDoubleTapTimeoutMs = config.iDoubleTapTimeoutMs;
  RCLCPP_DEBUG(m_logger, "[%s]   Double tap timeout (ms): %u", m_deviceNode.c_str(),
               m_configuration.iDoubleTapTimeoutMs);

  m_configuration.iButtonRepeatRateMs = config.iButtonRepeatRateMs;
  RCLCPP_DEBUG(m_logger, "[%s]   Button repeat rate (ms): %u", m_deviceNode.c_str(),
               m_configuration.iButtonRepeatRateMs);

  m_configuration.iButtonReleaseDelayMs = config.iButtonReleaseDelayMs;
  RCLCPP_DEBUG(m_logger, "[%s]   Button repeat delay (ms): %u", m_deviceNode.c_str(),
               m_configuration.iButtonReleaseDelayMs);

  m_configuration.bPowerOffOnStandby = config.bPowerOffOnStandby;
  RCLCPP_DEBUG(m_logger, "[%s]   Power off on standby: %s", m_deviceNode.c_str(),
               (m_configuration.bPowerOffOnStandby == 1 ? "true" : "false"));

  m_configuration.iFirmwareVersion = config.iFirmwareVersion;
  RCLCPP_DEBUG(m_logger, "[%s]   Firmware version: %u", m_deviceNode.c_str(),
               m_configuration.iFirmwareVersion);

  std::memcpy(m_configuration.strDeviceLanguage, config.strDeviceLanguage, 3);
  RCLCPP_DEBUG(m_logger, "[%s]   Device language: %c%c%c", m_deviceNode.c_str(),
               m_configuration.strDeviceLanguage[0], m_configuration.strDeviceLanguage[1],
               m_configuration.strDeviceLanguage[2]);

  m_configuration.iFirmwareBuildDate = config.iFirmwareBuildDate;
  const std::time_t buildDate = static_cast<std::time_t>(m_configuration.iFirmwareBuildDate);
  RCLCPP_DEBUG(m_logger, "[%s]   Firmware build date: %s", m_deviceNode.c_str(),
               CecTranslator::TranslateBuildDate(buildDate).c_str());

  RCLCPP_DEBUG(m_logger, "[%s] Configuration updated by libCEC", m_deviceNode.c_str());
}

void CecAdapter::SetVendorFromCEC(const std::string& vendorName)
{
  m_vendorName = vendorName;
}

void CecAdapter::OnPowerOn()
{
  RCLCPP_INFO(m_logger, "[%s] TV was powered on", m_deviceNode.c_str());

  m_callback.OnPowerOn(*this);
}

void CecAdapter::OnStandby()
{
  RCLCPP_INFO(m_logger, "[%s] TV was powered off", m_deviceNode.c_str());

  m_callback.OnPowerOff(*this);
}

void CecAdapter::ReopenConnection(bool bAsync)
{
  RCLCPP_INFO(m_logger, "[%s] Restarting connection (%s)", m_deviceNode.c_str(),
              (bAsync ? "async" : "sync"));

  if (bAsync)
  {
    // TODO: Async reopen not implemented
    RCLCPP_ERROR(m_logger, "[%s] Restart async not implemented", m_deviceNode.c_str());

    return;
  }

  // Reset all members to their defaults
  Deinitialize();

  // Reopen the connection
  Initialize();
}

void CecAdapter::CecLogMessage(const ::CEC::cec_log_message* message)
{
  std::lock_guard<std::mutex> lock(m_mutex);

  switch (message->level)
  {
    case ::CEC::CEC_LOG_ERROR:
      RCLCPP_DEBUG(m_logger, "[%s] %s", m_deviceNode.c_str(), message->message);
      break;
    case ::CEC::CEC_LOG_WARNING:
      RCLCPP_WARN(m_logger, "[%s] %s", m_deviceNode.c_str(), message->message);
      break;
    case ::CEC::CEC_LOG_NOTICE:
      RCLCPP_INFO(m_logger, "[%s] %s", m_deviceNode.c_str(), message->message);
      break;
    case ::CEC::CEC_LOG_TRAFFIC:
      // Ignored
      break;
    case ::CEC::CEC_LOG_DEBUG:
      RCLCPP_DEBUG(m_logger, "[%s] %s", m_deviceNode.c_str(), message->message);
      break;
    case ::CEC::CEC_LOG_ALL:
    default:
      RCLCPP_INFO(m_logger, "[%s] %s", m_deviceNode.c_str(), message->message);
      break;
  }
}

void CecAdapter::CecKeyPress(const ::CEC::cec_keypress* key)
{
  std::lock_guard<std::mutex> lock(m_mutex);

  RCLCPP_INFO(m_logger, "[%s] Received key press: %s (%u ms)", m_deviceNode.c_str(),
              CecTranslator::TranslateKeyCode(key->keycode).c_str(), key->duration);
}

void CecAdapter::CecCommand(const ::CEC::cec_command* command)
{
  std::lock_guard<std::mutex> lock(m_mutex);

  RCLCPP_INFO(m_logger, "[%s] Received command: %s", m_deviceNode.c_str(),
              CecTranslator::TranslateOpcode(command->opcode).c_str());
  RCLCPP_INFO(m_logger, "[%s]   Initiator: %s", m_deviceNode.c_str(),
              CecTranslator::TranslateLogicalAddress(command->initiator).c_str());
  RCLCPP_INFO(m_logger, "[%s]   Destination: %s", m_deviceNode.c_str(),
              CecTranslator::TranslateLogicalAddress(command->destination).c_str());
  RCLCPP_INFO(m_logger, "[%s]   Parameters: %s", m_deviceNode.c_str(),
              CecUtils::ParametersToHexArray(command->parameters).c_str());

  switch (command->opcode)
  {
    case ::CEC::CEC_OPCODE_REPORT_PHYSICAL_ADDRESS:
    {
      // This seems to be emitted when Vizio TVs are powered on
      OnPowerOn();

      break;
    }
    case ::CEC::CEC_OPCODE_SET_MENU_LANGUAGE:
    {
      // This seems to be emitted when Toshiba TVs are powered on
      OnPowerOn();

      break;
    }
    case ::CEC::CEC_OPCODE_ROUTING_CHANGE:
    {
      // This seems to be emitted when CEC-compliant TVs are powered on
      OnPowerOn();

      break;
    }
    case ::CEC::CEC_OPCODE_DEVICE_VENDOR_ID:
    {
      if (command->parameters.size != 3)
      {
        RCLCPP_ERROR(m_logger, "[%s] Expected 3 parameters", m_deviceNode.c_str());
      }
      else
      {
        const ::CEC::cec_vendor_id vendorId = static_cast<::CEC::cec_vendor_id>(
            command->parameters[0] << 16 | command->parameters[1] << 8 | command->parameters[2]);

        const std::string vendorName = CecTranslator::TranslateVendorID(vendorId);

        RCLCPP_INFO(m_logger, "[%s]   Vendor: %s", m_deviceNode.c_str(),
                    !vendorName.empty() ? vendorName.c_str() : "unknown");

        SetVendorFromCEC(vendorName);
      }

      break;
    }
    case ::CEC::CEC_OPCODE_STANDBY:
    {
      // TV turned off
      OnStandby();

      break;
    }
    default:
      break;
  }
}

void CecAdapter::CecConfiguration(const ::CEC::libcec_configuration* config)
{
  std::lock_guard<std::mutex> lock(m_mutex);

  SetConfigurationFromLibCEC(*config);
}

void CecAdapter::CecAlert(const ::CEC::libcec_alert alert, const ::CEC::libcec_parameter data)
{
  // Ignore poll failures
  if (alert == ::CEC::CEC_ALERT_TV_POLL_FAILED)
    return;

  std::lock_guard<std::mutex> lock(m_mutex);

  RCLCPP_ERROR(m_logger, "[%s] CEC alert: %s", m_deviceNode.c_str(),
               CecTranslator::TranslateAlert(alert).c_str());

  bool bReopenConnection = false;
  std::string alertString;
  switch (alert)
  {
    case ::CEC::CEC_ALERT_SERVICE_DEVICE:
      alertString = "This device needs servicing";
      break;
    case ::CEC::CEC_ALERT_CONNECTION_LOST:
      bReopenConnection = true;
      alertString = "Connection lost";
      break;
    case ::CEC::CEC_ALERT_PERMISSION_ERROR:
      bReopenConnection = true;
      alertString = "This user does not have permissions to open the CEC adapter";
      break;
    case ::CEC::CEC_ALERT_PORT_BUSY:
      bReopenConnection = true;
      alertString = "The port is busy. Only one program can access the CEC adapter";
      break;
    default:
      break;
  }

  // Display the alert
  if (!alertString.empty())
  {
    if (data.paramType == ::CEC::CEC_PARAMETER_TYPE_STRING && data.paramData != nullptr)
    {
      alertString += " - ";
      alertString += static_cast<const char*>(data.paramData);
    }
    RCLCPP_ERROR(m_logger, "[%s] %s", m_deviceNode.c_str(), alertString.c_str());
  }

  if (bReopenConnection)
  {
    // Reopen the connection asynchronously. Otherwise a deadlock may occur.
    // Reconnect means destruction and recreation of our libcec instance, but
    // libCEC calls this callback function synchronously and must not be
    // destroyed meanwhile.
    ReopenConnection(true);
  }
}

int CecAdapter::CecMenuStateChanged(const ::CEC::cec_menu_state state)
{
  std::lock_guard<std::mutex> lock(m_mutex);

  RCLCPP_INFO(m_logger, "[%s] CEC menu state changed: %s", m_deviceNode.c_str(),
              (state == ::CEC::CEC_MENU_STATE_ACTIVATED ? "activated" : "deactivated"));

  return 1;
}

void CecAdapter::CecSourceActivated(const ::CEC::cec_logical_address address,
                                    const uint8_t activated)
{
  std::lock_guard<std::mutex> lock(m_mutex);

  RCLCPP_INFO(m_logger, "[%s] Source %s: %s", m_deviceNode.c_str(),
              (activated == 1 ? "activated" : "deactivated"),
              CecTranslator::TranslateLogicalAddress(address).c_str());
}

void CecAdapter::CecLogMessage(void* cbParam, const ::CEC::cec_log_message* message)
{
  CecAdapter* adapter = static_cast<CecAdapter*>(cbParam);
  if (adapter == nullptr)
    return;

  adapter->CecLogMessage(message);
}

void CecAdapter::CecKeyPress(void* cbParam, const ::CEC::cec_keypress* key)
{
  CecAdapter* adapter = static_cast<CecAdapter*>(cbParam);
  if (adapter == nullptr)
    return;

  adapter->CecKeyPress(key);
}

void CecAdapter::CecCommand(void* cbParam, const ::CEC::cec_command* command)
{
  CecAdapter* adapter = static_cast<CecAdapter*>(cbParam);
  if (adapter == nullptr)
    return;

  adapter->CecCommand(command);
}

void CecAdapter::CecConfiguration(void* cbParam, const ::CEC::libcec_configuration* config)
{
  CecAdapter* adapter = static_cast<CecAdapter*>(cbParam);
  if (adapter == nullptr)
    return;

  adapter->CecConfiguration(config);
}

void CecAdapter::CecAlert(void* cbParam,
                          const ::CEC::libcec_alert alert,
                          const ::CEC::libcec_parameter data)
{
  CecAdapter* adapter = static_cast<CecAdapter*>(cbParam);
  if (adapter == nullptr)
    return;

  adapter->CecAlert(alert, data);
}

int CecAdapter::CecMenuStateChanged(void* cbParam, const ::CEC::cec_menu_state state)
{
  CecAdapter* adapter = static_cast<CecAdapter*>(cbParam);
  if (adapter == nullptr)
    return 0;

  return adapter->CecMenuStateChanged(state);
}

void CecAdapter::CecSourceActivated(void* cbParam,
                                    const ::CEC::cec_logical_address address,
                                    const uint8_t activated)
{
  CecAdapter* adapter = static_cast<CecAdapter*>(cbParam);
  if (adapter == nullptr)
    return;

  adapter->CecSourceActivated(address, activated);
}
