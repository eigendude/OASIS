/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "Ssd1305Transport.hpp"

#include <cerrno>
#include <stdexcept>
#include <string>
#include <system_error>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

using OASIS::Display::LinuxI2cTransport;

LinuxI2cTransport::~LinuxI2cTransport() noexcept
{
  Close();
}

void LinuxI2cTransport::Open(const std::string& device, int address)
{
  if (IsOpen())
    return;

  m_descriptor = open(device.c_str(), O_RDWR | O_CLOEXEC);
  if (m_descriptor < 0)
    throw std::system_error(errno, std::generic_category(), "open(" + device + ")");

  if (ioctl(m_descriptor, I2C_SLAVE, static_cast<unsigned long>(address)) < 0)
  {
    const int error_number = errno;
    Close();
    throw std::system_error(error_number, std::generic_category(), "I2C_SLAVE " + device);
  }
}

void LinuxI2cTransport::Close() noexcept
{
  if (m_descriptor >= 0)
  {
    close(m_descriptor);
    m_descriptor = -1;
  }
}

void LinuxI2cTransport::Write(std::span<const std::uint8_t> transaction)
{
  const ssize_t result = write(m_descriptor, transaction.data(), transaction.size());
  if (result < 0)
  {
    const int error_number = errno;
    throw std::system_error(error_number, std::generic_category(), "write SSD1305 I2C transaction");
  }

  if (static_cast<std::size_t>(result) != transaction.size())
  {
    throw std::runtime_error("short SSD1305 I2C write: expected " +
                             std::to_string(transaction.size()) + " bytes but wrote " +
                             std::to_string(result));
  }
}
