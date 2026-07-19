/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "I2cRegisterDevice.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

#include <linux/i2c.h>

namespace OASIS::PowerMeter
{
// ACS37800-DS Rev. 4, Figure 30: every register read returns four bytes
constexpr std::size_t ACS37800_REGISTER_WIDTH = 4;

// ACS37800-DS Rev. 4, Figure 30: selector write plus repeated-start read
constexpr std::size_t ACS37800_REGISTER_READ_MESSAGE_COUNT = 2;

/** \brief Storage for one combined ACS37800 I2C register read */
struct I2cRegisterReadTransfer
{
  /** \brief Build the write-selector and repeated-start read messages */
  I2cRegisterReadTransfer(int device_address, std::uint8_t register_address);

  I2cRegisterReadTransfer(const I2cRegisterReadTransfer&) = delete;
  I2cRegisterReadTransfer& operator=(const I2cRegisterReadTransfer&) = delete;
  I2cRegisterReadTransfer(I2cRegisterReadTransfer&&) = delete;
  I2cRegisterReadTransfer& operator=(I2cRegisterReadTransfer&&) = delete;

  //! One-byte register selector written before the repeated start
  std::uint8_t register_selector;

  //! Four register bytes returned least-significant byte first
  std::array<std::uint8_t, ACS37800_REGISTER_WIDTH> read_data{};

  //! Linux messages passed together in one I2C_RDWR ioctl
  std::array<i2c_msg, ACS37800_REGISTER_READ_MESSAGE_COUNT> messages{};
};

/** \brief Decode the ACS37800's least-significant-byte-first register data */
std::uint32_t DecodeAcs37800RegisterBytes(
    const std::array<std::uint8_t, ACS37800_REGISTER_WIDTH>& bytes);

/** \brief Encode a selector and host-order value for an ACS37800 write */
std::array<std::uint8_t, ACS37800_REGISTER_WIDTH + 1> EncodeAcs37800RegisterWrite(
    std::uint8_t address, std::uint32_t value);

/** \brief Throw when I2C_RDWR fails or completes a short transaction */
void ValidateI2cTransferResult(int transferred_messages,
                               int error_number,
                               const std::string& device_path);

/** \brief Persistent Linux I2C_RDWR register transport */
class LinuxI2cRegisterDevice : public II2cRegisterDevice
{
public:
  LinuxI2cRegisterDevice(std::string device_path, int device_address);
  ~LinuxI2cRegisterDevice() override;

  LinuxI2cRegisterDevice(const LinuxI2cRegisterDevice&) = delete;
  LinuxI2cRegisterDevice& operator=(const LinuxI2cRegisterDevice&) = delete;
  LinuxI2cRegisterDevice(LinuxI2cRegisterDevice&&) = delete;
  LinuxI2cRegisterDevice& operator=(LinuxI2cRegisterDevice&&) = delete;

  std::uint32_t ReadRegister(std::uint8_t address) override;
  void WriteRegister(std::uint8_t address, std::uint32_t value) override;

private:
  std::string m_devicePath;
  int m_deviceAddress;
  int m_descriptor{-1};
};
} // namespace OASIS::PowerMeter
