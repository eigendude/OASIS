/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "display/Ssd1305Display.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <thread>

#include <fcntl.h>
#include <linux/gpio.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace OASIS::Display
{
namespace
{
constexpr std::uint8_t kControlCommand = 0x00;
constexpr std::uint8_t kControlData = 0x40;
constexpr std::uint8_t kDisplayOff = 0xae;
constexpr std::uint8_t kDisplayOn = 0xaf;
constexpr std::uint8_t kSetContrast = 0x81;
constexpr std::uint8_t kSetColumnAddress = 0x21;
constexpr std::uint8_t kSetPageAddress = 0x22;
constexpr std::uint8_t kMaximumControllerColumn = 131;
constexpr std::size_t kMaximumI2cPayload = 32;
constexpr const char* kGpioConsumer = "oasis_ssd1305_display";

std::string ErrnoMessage(const char* operation)
{
  return std::string(operation) + ": " + std::strerror(errno);
}
} // namespace

Ssd1305Display::~Ssd1305Display()
{
  Close();
}

std::size_t Ssd1305Display::FramebufferSize() const
{
  return static_cast<std::size_t>(m_config.width) * (m_config.height / 8U);
}

bool Ssd1305Display::Open(const Ssd1305Config& config, std::string& error)
{
  Close();
  m_config = config;

  if (m_config.width == 0 || m_config.height == 0 ||
      (m_config.height % 8U) != 0U || m_config.width > 128 ||
      m_config.height > 64)
  {
    error = "unsupported SSD1305 geometry";
    return false;
  }

  if (static_cast<unsigned int>(m_config.columnOffset) + m_config.width - 1U >
      kMaximumControllerColumn)
  {
    error = "column offset exceeds SSD1305 controller RAM";
    return false;
  }

  m_i2cFd = ::open(m_config.i2cDevice.c_str(), O_RDWR | O_CLOEXEC);
  if (m_i2cFd < 0)
  {
    error = ErrnoMessage("open I2C device");
    return false;
  }

  if (::ioctl(m_i2cFd, I2C_SLAVE, m_config.i2cAddress) < 0)
  {
    error = ErrnoMessage("select I2C slave");
    Close();
    return false;
  }

  if (!HardwareReset(error) || !Initialize(error))
  {
    Close();
    return false;
  }

  m_lastFramebuffer.assign(FramebufferSize(), 0xff);
  return Clear(error);
}

void Ssd1305Display::Close()
{
  if (m_i2cFd >= 0 && m_config.blankOnClose)
  {
    std::string ignored;
    Clear(ignored);
    SetEnabled(false, ignored);
  }

  if (m_gpioLineFd >= 0)
  {
    ::close(m_gpioLineFd);
    m_gpioLineFd = -1;
  }
  if (m_gpioChipFd >= 0)
  {
    ::close(m_gpioChipFd);
    m_gpioChipFd = -1;
  }
  if (m_i2cFd >= 0)
  {
    ::close(m_i2cFd);
    m_i2cFd = -1;
  }

  m_lastFramebuffer.clear();
}

bool Ssd1305Display::HardwareReset(std::string& error)
{
  m_gpioChipFd = ::open(m_config.gpioChip.c_str(), O_RDONLY | O_CLOEXEC);
  if (m_gpioChipFd < 0)
  {
    error = ErrnoMessage("open GPIO chip");
    return false;
  }

  gpio_v2_line_request request{};
  request.offsets[0] = m_config.resetGpio;
  request.num_lines = 1;
  request.config.flags = GPIO_V2_LINE_FLAG_OUTPUT;
  std::strncpy(request.consumer, kGpioConsumer, sizeof(request.consumer) - 1);

  if (::ioctl(m_gpioChipFd, GPIO_V2_GET_LINE_IOCTL, &request) < 0)
  {
    error = ErrnoMessage("request reset GPIO");
    return false;
  }
  m_gpioLineFd = request.fd;

  auto setValue = [this, &error](bool high) {
    gpio_v2_line_values values{};
    values.mask = 1U;
    values.bits = high ? 1U : 0U;
    if (::ioctl(m_gpioLineFd, GPIO_V2_LINE_SET_VALUES_IOCTL, &values) < 0)
    {
      error = ErrnoMessage("set reset GPIO");
      return false;
    }
    return true;
  };

  if (!setValue(true))
    return false;
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  if (!setValue(false))
    return false;
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  if (!setValue(true))
    return false;
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  return true;
}

bool Ssd1305Display::Initialize(std::string& error)
{
  // SSD1305 sequence used by Adafruit's Product 4567 driver. This is not the
  // SSD1306 initialization sequence. The panel uses a four-column RAM offset.
  const std::vector<std::uint8_t> commands{
      kDisplayOff,
      0xd5, 0x80,       // Display clock divide
      0xa1,             // Segment remap
      0xa8, static_cast<std::uint8_t>(m_config.height - 1U),
      0xd3, 0x00,       // Display offset
      0xad, 0x8e,       // Master configuration
      0xd8, 0x05,       // Area color/low-power mode
      0x20, 0x00,       // Horizontal addressing mode
      0x40,             // Display start line
      0x2e,             // Stop scrolling
      0xc8,             // COM scan direction
      0xda, 0x12,       // COM pin configuration
      0x91, 0x3f, 0x3f, 0x3f, 0x3f,
      kSetContrast, m_config.contrast,
      0xd9, 0xd2,       // Precharge period
      0xdb, 0x34,       // VCOM deselect
      0xa6,             // Normal display
      0xa4,             // Follow display RAM
      0x8d, 0x14,       // Internal charge pump
      kDisplayOn,
  };

  return WriteCommands(commands, error);
}

bool Ssd1305Display::WriteCommand(std::uint8_t command, std::string& error)
{
  const std::uint8_t packet[]{kControlCommand, command};
  const ssize_t written = ::write(m_i2cFd, packet, sizeof(packet));
  if (written != static_cast<ssize_t>(sizeof(packet)))
  {
    error = written < 0 ? ErrnoMessage("write SSD1305 command")
                        : "short SSD1305 command write";
    return false;
  }
  return true;
}

bool Ssd1305Display::WriteCommands(
    const std::vector<std::uint8_t>& commands, std::string& error)
{
  for (const std::uint8_t command : commands)
  {
    if (!WriteCommand(command, error))
      return false;
  }
  return true;
}

bool Ssd1305Display::WriteData(const std::uint8_t* data, std::size_t size,
                               std::string& error)
{
  std::vector<std::uint8_t> packet;
  packet.reserve(kMaximumI2cPayload + 1U);

  while (size > 0)
  {
    const std::size_t chunkSize = std::min(size, kMaximumI2cPayload);
    packet.clear();
    packet.push_back(kControlData);
    packet.insert(packet.end(), data, data + chunkSize);

    const ssize_t written = ::write(m_i2cFd, packet.data(), packet.size());
    if (written != static_cast<ssize_t>(packet.size()))
    {
      error = written < 0 ? ErrnoMessage("write SSD1305 data")
                          : "short SSD1305 data write";
      return false;
    }

    data += chunkSize;
    size -= chunkSize;
  }
  return true;
}

bool Ssd1305Display::SetWindow(std::uint8_t firstPage,
                               std::uint8_t lastPage,
                               std::string& error)
{
  const std::uint8_t firstColumn = m_config.columnOffset;
  const std::uint8_t lastColumn = static_cast<std::uint8_t>(
      m_config.columnOffset + m_config.width - 1U);
  return WriteCommands({kSetColumnAddress, firstColumn, lastColumn,
                        kSetPageAddress, firstPage, lastPage}, error);
}

bool Ssd1305Display::WriteFramebuffer(
    const std::vector<std::uint8_t>& framebuffer, std::string& error)
{
  if (!IsOpen())
  {
    error = "SSD1305 display is not open";
    return false;
  }
  if (framebuffer.size() != FramebufferSize())
  {
    error = "invalid SSD1305 framebuffer size";
    return false;
  }

  const std::size_t pageSize = m_config.width;
  const std::size_t pageCount = m_config.height / 8U;
  for (std::size_t page = 0; page < pageCount; ++page)
  {
    const auto begin = framebuffer.begin() + page * pageSize;
    const auto previous = m_lastFramebuffer.begin() + page * pageSize;
    if (std::equal(begin, begin + pageSize, previous))
      continue;

    if (!SetWindow(static_cast<std::uint8_t>(page),
                   static_cast<std::uint8_t>(page), error) ||
        !WriteData(&framebuffer[page * pageSize], pageSize, error))
    {
      return false;
    }

    std::copy(begin, begin + pageSize,
              m_lastFramebuffer.begin() + page * pageSize);
  }

  return true;
}

bool Ssd1305Display::Clear(std::string& error)
{
  return WriteFramebuffer(std::vector<std::uint8_t>(FramebufferSize(), 0),
                          error);
}

bool Ssd1305Display::SetEnabled(bool enabled, std::string& error)
{
  return WriteCommand(enabled ? kDisplayOn : kDisplayOff, error);
}

bool Ssd1305Display::SetContrast(std::uint8_t contrast, std::string& error)
{
  if (!WriteCommands({kSetContrast, contrast}, error))
    return false;
  m_config.contrast = contrast;
  return true;
}
} // namespace OASIS::Display
