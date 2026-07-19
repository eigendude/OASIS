/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "LinuxI2cRegisterDevice.hpp"

#include <cerrno>
#include <stdexcept>
#include <system_error>
#include <utility>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

using OASIS::PowerMeter::I2cRegisterReadTransfer;
using OASIS::PowerMeter::LinuxI2cRegisterDevice;

I2cRegisterReadTransfer::I2cRegisterReadTransfer(int device_address, std::uint8_t register_address)
  : register_selector(register_address)
{
  messages[0].addr = static_cast<__u16>(device_address);
  messages[0].flags = 0;
  messages[0].len = 1;
  messages[0].buf = &register_selector;

  messages[1].addr = static_cast<__u16>(device_address);
  messages[1].flags = I2C_M_RD;
  messages[1].len = static_cast<__u16>(read_data.size());
  messages[1].buf = read_data.data();
}

std::uint32_t OASIS::PowerMeter::DecodeAcs37800RegisterBytes(
    const std::array<std::uint8_t, ACS37800_REGISTER_WIDTH>& bytes)
{
  // ACS37800-DS Rev. 4, Figure 30 sends D[7:0] first and D[31:24] last
  return static_cast<std::uint32_t>(bytes[0]) | (static_cast<std::uint32_t>(bytes[1]) << 8U) |
         (static_cast<std::uint32_t>(bytes[2]) << 16U) |
         (static_cast<std::uint32_t>(bytes[3]) << 24U);
}

std::array<std::uint8_t, OASIS::PowerMeter::ACS37800_REGISTER_WIDTH + 1> OASIS::PowerMeter::
    EncodeAcs37800RegisterWrite(std::uint8_t address, std::uint32_t value)
{
  // ACS37800-DS Rev. 4, Figure 29 sends the selector followed by D[7:0]
  // through D[31:24]
  return {
      address,
      static_cast<std::uint8_t>(value),
      static_cast<std::uint8_t>(value >> 8U),
      static_cast<std::uint8_t>(value >> 16U),
      static_cast<std::uint8_t>(value >> 24U),
  };
}

void OASIS::PowerMeter::ValidateI2cTransferResult(int transferred_messages,
                                                  int error_number,
                                                  const std::string& device_path)
{
  if (transferred_messages < 0)
    throw std::system_error(error_number, std::generic_category(), "I2C_RDWR on " + device_path);
  if (transferred_messages != static_cast<int>(ACS37800_REGISTER_READ_MESSAGE_COUNT))
  {
    throw std::runtime_error("I2C_RDWR on " + device_path + " completed " +
                             std::to_string(transferred_messages) + " of " +
                             std::to_string(ACS37800_REGISTER_READ_MESSAGE_COUNT) + " messages");
  }
}

LinuxI2cRegisterDevice::LinuxI2cRegisterDevice(std::string device_path, int device_address)
  : m_devicePath(std::move(device_path)), m_deviceAddress(device_address)
{
  if (m_deviceAddress < 0x03 || m_deviceAddress > 0x77)
    throw std::invalid_argument("device address must be in [0x03, 0x77]");

  m_descriptor = open(m_devicePath.c_str(), O_RDWR | O_CLOEXEC);
  if (m_descriptor < 0)
    throw std::system_error(errno, std::generic_category(), "open(" + m_devicePath + ")");
}

LinuxI2cRegisterDevice::~LinuxI2cRegisterDevice()
{
  if (m_descriptor >= 0)
    close(m_descriptor);
}

std::uint32_t LinuxI2cRegisterDevice::ReadRegister(std::uint8_t address)
{
  I2cRegisterReadTransfer transfer(m_deviceAddress, address);
  i2c_rdwr_ioctl_data transaction{
      .msgs = transfer.messages.data(),
      .nmsgs = static_cast<__u32>(transfer.messages.size()),
  };

  const int transferred_messages = ioctl(m_descriptor, I2C_RDWR, &transaction);
  const int error_number = errno;
  ValidateI2cTransferResult(transferred_messages, error_number, m_devicePath);

  return DecodeAcs37800RegisterBytes(transfer.read_data);
}

void LinuxI2cRegisterDevice::WriteRegister(std::uint8_t address, std::uint32_t value)
{
  auto bytes = EncodeAcs37800RegisterWrite(address, value);
  i2c_msg message{
      .addr = static_cast<__u16>(m_deviceAddress),
      .flags = 0,
      .len = static_cast<__u16>(bytes.size()),
      .buf = bytes.data(),
  };
  i2c_rdwr_ioctl_data transaction{.msgs = &message, .nmsgs = 1};
  const int result = ioctl(m_descriptor, I2C_RDWR, &transaction);
  if (result < 0)
    throw std::system_error(errno, std::generic_category(),
                            "I2C write on " + m_devicePath + " address " +
                                std::to_string(m_deviceAddress) + " register " +
                                std::to_string(address));
  if (result != 1)
    throw std::runtime_error("I2C write on " + m_devicePath + " address " +
                             std::to_string(m_deviceAddress) + " register " +
                             std::to_string(address) + " was incomplete");
}
