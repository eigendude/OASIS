/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "Ssd1305Device.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

using OASIS::Display::Ssd1305Device;
using OASIS::Display::Ssd1305DeviceConfig;
using OASIS::Display::Ssd1305Framebuffer;

namespace
{
constexpr std::uint8_t CONTROL_COMMAND_STREAM = 0x00;
constexpr std::uint8_t CONTROL_DATA_STREAM = 0x40;
constexpr std::uint8_t DISPLAY_OFF = 0xAE;
constexpr std::uint8_t DISPLAY_ON = 0xAF;
constexpr std::uint8_t SET_MEMORY_ADDRESSING_MODE = 0x20;
constexpr std::uint8_t SET_COLUMN_ADDRESS = 0x21;
constexpr std::uint8_t SET_PAGE_ADDRESS = 0x22;
constexpr std::uint8_t SET_START_LINE = 0x40;
constexpr std::uint8_t SET_CONTRAST = 0x81;
constexpr std::uint8_t SET_CHARGE_PUMP = 0x8D;
constexpr std::uint8_t SET_LOOKUP_TABLE = 0x91;
constexpr std::uint8_t RESUME_TO_RAM = 0xA4;
constexpr std::uint8_t NORMAL_DISPLAY = 0xA6;
constexpr std::uint8_t SET_SEGMENT_REMAP_REVERSED = 0xA1;
constexpr std::uint8_t SET_MULTIPLEX_RATIO = 0xA8;
constexpr std::uint8_t SET_MASTER_CONFIGURATION = 0xAD;
constexpr std::uint8_t SET_COM_SCAN_DEC = 0xC8;
constexpr std::uint8_t SET_DISPLAY_OFFSET = 0xD3;
constexpr std::uint8_t SET_DISPLAY_CLOCK_DIVIDE = 0xD5;
constexpr std::uint8_t SET_AREA_COLOR = 0xD8;
constexpr std::uint8_t SET_PRECHARGE = 0xD9;
constexpr std::uint8_t SET_COM_PINS = 0xDA;
constexpr std::uint8_t SET_VCOMH_DESELECT = 0xDB;
constexpr std::size_t SSD1305_DATA_CHUNK_SIZE = 32;

// Time for the Linux I2C transport and unplugged controller to settle after
// closing the stale file descriptor
constexpr auto RECOVERY_TRANSPORT_RESET_DELAY = std::chrono::milliseconds(20);

std::string HexAddress(int address)
{
  std::ostringstream stream;
  stream << "0x" << std::hex << address;
  return stream.str();
}

} // namespace

Ssd1305Device::Ssd1305Device(Ssd1305DeviceConfig config)
  : Ssd1305Device(std::move(config), std::make_unique<OASIS::Display::LinuxI2cTransport>())
{
}

Ssd1305Device::Ssd1305Device(Ssd1305DeviceConfig config,
                             std::unique_ptr<OASIS::Display::Ssd1305Transport> transport)
  : m_config(std::move(config)), m_transport(std::move(transport))
{
  if (!m_transport)
    throw std::invalid_argument("SSD1305 transport must not be null");
  ValidateConfig();
}

Ssd1305Device::~Ssd1305Device() noexcept
{
  Close();
}

void Ssd1305Device::Open()
{
  if (IsOpen())
    return;

  m_transport->Open(m_config.i2c_device, m_config.i2c_address);
}

void Ssd1305Device::Close() noexcept
{
  m_transport->Close();
}

bool Ssd1305Device::IsOpen() const noexcept
{
  return m_transport->IsOpen();
}

void Ssd1305Device::RecoverTransport()
{
  Close();
  std::this_thread::sleep_for(RECOVERY_TRANSPORT_RESET_DELAY);
  Open();
}

void Ssd1305Device::Initialize()
{
  Open();

  // Product 4567 exposes 128 of the SSD1305's 132 GDDRAM columns. The
  // verified Adafruit SSD1305 drivers use a four-column panel offset.
  //
  // The controller is not host-resettable in the Qwiic installation, so this
  // sequence explicitly sets addressing, timing, drive, and charge-pump state
  // instead of relying on power-on defaults.
  const std::array<std::uint8_t, 30> init{
      DISPLAY_OFF,
      SET_DISPLAY_CLOCK_DIVIDE,
      0x80,
      SET_MULTIPLEX_RATIO,
      static_cast<std::uint8_t>(m_config.height - 1U),
      SET_DISPLAY_OFFSET,
      0x00,
      SET_START_LINE | 0x00,
      SET_CHARGE_PUMP,
      0x14,
      SET_MEMORY_ADDRESSING_MODE,
      0x00,
      0x2E,
      SET_COM_PINS,
      0x12,
      SET_MASTER_CONFIGURATION,
      0x8E,
      SET_AREA_COLOR,
      0x05,
      SET_LOOKUP_TABLE,
      0x3F,
      0x3F,
      0x3F,
      0x3F,
      SET_PRECHARGE,
      // Adafruit_CircuitPython_SSD1305 uses 0xD2 for the SSD1305 pre-charge
      // period; 0xF1 is the SSD1306-oriented value used by older
      // implementations
      0xD2,
      SET_VCOMH_DESELECT,
      0x34,
      RESUME_TO_RAM,
      NORMAL_DISPLAY,
  };
  WriteCommands(init);
  ConfigureOrientation();
  ConfigureAddressing();
}

void Ssd1305Device::ConfigureOrientation()
{
  WriteCommands(std::array<std::uint8_t, 1>{
      SET_SEGMENT_REMAP_REVERSED,
  });
  WriteCommands(std::array<std::uint8_t, 1>{
      SET_COM_SCAN_DEC,
  });
}

void Ssd1305Device::ConfigureAddressing()
{
  const std::array<std::uint8_t, 8> commands{
      SET_MEMORY_ADDRESSING_MODE,
      0x00,
      SET_COLUMN_ADDRESS,
      m_config.column_offset,
      static_cast<std::uint8_t>(m_config.column_offset + m_config.width - 1U),
      SET_PAGE_ADDRESS,
      0x00,
      static_cast<std::uint8_t>(SSD1305_PAGE_COUNT - 1U),
  };
  WriteCommands(commands);
}

void Ssd1305Device::SetDisplayEnabled(bool enabled)
{
  const std::array<std::uint8_t, 1> command{enabled ? DISPLAY_ON : DISPLAY_OFF};
  WriteCommands(command);
}

void Ssd1305Device::SetContrast(std::uint8_t contrast)
{
  const std::array<std::uint8_t, 2> command{SET_CONTRAST, contrast};
  WriteCommands(command);
  m_config.contrast = contrast;
}

void Ssd1305Device::WriteFullFrame(const Ssd1305Framebuffer::Buffer& framebuffer)
{
  Open();
  for (std::size_t page = 0; page < SSD1305_PAGE_COUNT; ++page)
    WritePage(framebuffer, page);
}

void Ssd1305Device::WritePage(const Ssd1305Framebuffer::Buffer& framebuffer, std::size_t page)
{
  if (page >= SSD1305_PAGE_COUNT)
    throw std::out_of_range("SSD1305 page index is out of range");

  SelectPage(page, 0, m_config.width - 1U);
  const auto begin =
      framebuffer.begin() + static_cast<std::ptrdiff_t>(page * SSD1305_DISPLAY_WIDTH);
  WriteData(std::span<const std::uint8_t>(begin, m_config.width));
}

void Ssd1305Device::RestoreAfterInitialization(const Ssd1305Framebuffer::Buffer& framebuffer,
                                               std::uint8_t contrast)
{
  Initialize();
  SetContrast(contrast);
  WriteFullFrame(framebuffer);
}

void Ssd1305Device::RestoreFramebufferState(const Ssd1305Framebuffer::Buffer& framebuffer,
                                            bool enabled)
{
  ConfigureOrientation();
  ConfigureAddressing();
  WriteFullFrame(framebuffer);
  if (enabled)
    SetDisplayEnabled(true);
}

void Ssd1305Device::ValidateConfig() const
{
  if (m_config.i2c_device.empty())
    throw std::invalid_argument("SSD1305 I2C device path must not be empty");
  if (m_config.i2c_address < 0x03 || m_config.i2c_address > 0x77)
    throw std::invalid_argument("SSD1305 I2C address must be in [0x03, 0x77]");
  if (m_config.width != SSD1305_DISPLAY_WIDTH || m_config.height != SSD1305_DISPLAY_HEIGHT)
    throw std::invalid_argument("SSD1305 driver currently supports only 128 x 32 panels");
  if (m_config.column_offset + m_config.width > 132)
    throw std::invalid_argument("SSD1305 column offset exceeds 132-column GDDRAM");
}

void Ssd1305Device::WriteCommands(std::span<const std::uint8_t> commands)
{
  Open();
  std::vector<std::uint8_t> transfer;
  transfer.reserve(commands.size() + 1U);
  transfer.push_back(CONTROL_COMMAND_STREAM);
  transfer.insert(transfer.end(), commands.begin(), commands.end());
  WriteTransaction(transfer, "write SSD1305 commands");
}

void Ssd1305Device::WriteData(std::span<const std::uint8_t> data)
{
  Open();
  for (std::size_t offset = 0; offset < data.size(); offset += SSD1305_DATA_CHUNK_SIZE)
  {
    const std::size_t chunk_size = std::min(SSD1305_DATA_CHUNK_SIZE, data.size() - offset);
    std::vector<std::uint8_t> transfer;
    transfer.reserve(chunk_size + 1U);
    transfer.push_back(CONTROL_DATA_STREAM);
    transfer.insert(transfer.end(), data.begin() + static_cast<std::ptrdiff_t>(offset),
                    data.begin() + static_cast<std::ptrdiff_t>(offset + chunk_size));
    WriteTransaction(transfer, "write SSD1305 display data");
  }
}

void Ssd1305Device::WriteTransaction(std::span<const std::uint8_t> transaction,
                                     const std::string& operation)
{
  Open();
  const std::string context =
      operation + " to " + m_config.i2c_device + " at address " + HexAddress(m_config.i2c_address);
  try
  {
    m_transport->Write(transaction);
  }
  catch (const std::system_error& error)
  {
    throw std::system_error(error.code(), context + ": " + error.what());
  }
  catch (const std::exception& error)
  {
    throw std::runtime_error(context + ": " + error.what());
  }
}

void Ssd1305Device::SelectPage(std::size_t page, std::size_t start_column, std::size_t end_column)
{
  const std::size_t absolute_start = m_config.column_offset + start_column;
  const std::size_t absolute_end = m_config.column_offset + end_column;
  const std::array<std::uint8_t, 6> commands{
      SET_COLUMN_ADDRESS,
      static_cast<std::uint8_t>(absolute_start),
      static_cast<std::uint8_t>(absolute_end),
      SET_PAGE_ADDRESS,
      static_cast<std::uint8_t>(page),
      static_cast<std::uint8_t>(page),
  };
  WriteCommands(commands);
}
