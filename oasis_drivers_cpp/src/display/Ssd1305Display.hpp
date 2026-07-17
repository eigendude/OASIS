/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace OASIS::Display
{
struct Ssd1305Config
{
  std::string i2cDevice{"/dev/i2c-1"};
  std::uint8_t i2cAddress{0x3c};
  std::string gpioChip{"/dev/gpiochip0"};
  std::uint32_t resetGpio{4};
  std::uint32_t width{128};
  std::uint32_t height{32};
  std::uint8_t columnOffset{4};
  std::uint8_t contrast{0xff};
  bool blankOnClose{true};
};

class Ssd1305Display
{
public:
  Ssd1305Display() = default;
  ~Ssd1305Display();

  Ssd1305Display(const Ssd1305Display&) = delete;
  Ssd1305Display& operator=(const Ssd1305Display&) = delete;

  bool Open(const Ssd1305Config& config, std::string& error);
  void Close();

  bool IsOpen() const { return m_i2cFd >= 0; }
  bool SetEnabled(bool enabled, std::string& error);
  bool SetContrast(std::uint8_t contrast, std::string& error);
  bool Clear(std::string& error);
  bool WriteFramebuffer(const std::vector<std::uint8_t>& framebuffer,
                        std::string& error);

  std::size_t FramebufferSize() const;

private:
  bool HardwareReset(std::string& error);
  bool WriteCommand(std::uint8_t command, std::string& error);
  bool WriteCommands(const std::vector<std::uint8_t>& commands,
                     std::string& error);
  bool WriteData(const std::uint8_t* data, std::size_t size,
                 std::string& error);
  bool Initialize(std::string& error);
  bool SetWindow(std::uint8_t firstPage, std::uint8_t lastPage,
                 std::string& error);

  Ssd1305Config m_config;
  int m_i2cFd{-1};
  int m_gpioChipFd{-1};
  int m_gpioLineFd{-1};
  std::vector<std::uint8_t> m_lastFramebuffer;
};
} // namespace OASIS::Display
