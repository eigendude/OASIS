/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <cstdint>
#include <span>
#include <string>

namespace OASIS::Display
{
/** \brief Transaction transport used by the ROS-free SSD1305 device */
class Ssd1305Transport
{
public:
  virtual ~Ssd1305Transport() = default;

  virtual void Open(const std::string& device, int address) = 0;
  virtual void Close() noexcept = 0;
  virtual bool IsOpen() const noexcept = 0;
  virtual void Write(std::span<const std::uint8_t> transaction) = 0;
};

/** \brief Linux i2c-dev transport for one selected seven-bit slave */
class LinuxI2cTransport final : public Ssd1305Transport
{
public:
  ~LinuxI2cTransport() noexcept override;

  void Open(const std::string& device, int address) override;
  void Close() noexcept override;
  bool IsOpen() const noexcept override { return m_descriptor >= 0; }
  void Write(std::span<const std::uint8_t> transaction) override;

private:
  int m_descriptor{-1};
};
} // namespace OASIS::Display
