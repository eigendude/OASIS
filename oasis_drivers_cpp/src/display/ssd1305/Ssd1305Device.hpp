/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "Ssd1305Framebuffer.hpp"
#include "Ssd1305Transport.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>

namespace OASIS::Display
{
/** \brief Linux I2C configuration for one SSD1305 display */
struct Ssd1305DeviceConfig
{
  /** \brief Linux I2C adapter device path, typically /dev/i2c-1 */
  std::string i2c_device;

  /** \brief Seven-bit SSD1305 I2C address, expected range [0x03, 0x77] */
  int i2c_address;

  /** \brief Visible panel width in pixels; Product 4567 uses 128 */
  std::size_t width;

  /** \brief Visible panel height in pixels; Product 4567 uses 32 */
  std::size_t height;

  /** \brief GDDRAM column offset; Adafruit Product 4567 uses 4 of 132 */
  std::uint8_t column_offset;

  /** \brief Contrast command argument in [0, 255] sent after init */
  std::uint8_t contrast;
};

/** \brief Hardware operations required by the SSD1305 ROS node */
class Ssd1305DeviceInterface
{
public:
  virtual ~Ssd1305DeviceInterface() = default;

  virtual void Initialize() = 0;
  virtual void SetDisplayEnabled(bool enabled) = 0;
  virtual void SetContrast(std::uint8_t contrast) = 0;
  virtual void WriteFullFrame(const Ssd1305Framebuffer::Buffer& framebuffer) = 0;
  virtual void WritePage(const Ssd1305Framebuffer::Buffer& framebuffer, std::size_t page) = 0;
  virtual void Recover(const Ssd1305Framebuffer::Buffer& framebuffer, bool enabled) = 0;
};

/** \brief Linux I2C SSD1305 command and framebuffer transport */
class Ssd1305Device : public Ssd1305DeviceInterface
{
public:
  explicit Ssd1305Device(Ssd1305DeviceConfig config);
  Ssd1305Device(Ssd1305DeviceConfig config, std::unique_ptr<Ssd1305Transport> transport);
  ~Ssd1305Device() noexcept override;

  Ssd1305Device(const Ssd1305Device&) = delete;
  Ssd1305Device& operator=(const Ssd1305Device&) = delete;
  Ssd1305Device(Ssd1305Device&&) = delete;
  Ssd1305Device& operator=(Ssd1305Device&&) = delete;

  void Open();
  void Close() noexcept;
  bool IsOpen() const noexcept;

  void Initialize() override;
  void SetDisplayEnabled(bool enabled) override;
  void SetContrast(std::uint8_t contrast) override;
  void WriteFullFrame(const Ssd1305Framebuffer::Buffer& framebuffer) override;
  void WritePage(const Ssd1305Framebuffer::Buffer& framebuffer, std::size_t page) override;
  void Recover(const Ssd1305Framebuffer::Buffer& framebuffer, bool enabled) override;

private:
  void ValidateConfig() const;
  void WriteCommands(std::span<const std::uint8_t> commands);
  void WriteData(std::span<const std::uint8_t> data);
  void WriteTransaction(std::span<const std::uint8_t> transaction, const std::string& operation);
  void SelectPage(std::size_t page, std::size_t start_column, std::size_t end_column);

  Ssd1305DeviceConfig m_config;
  std::unique_ptr<Ssd1305Transport> m_transport;
};
} // namespace OASIS::Display
