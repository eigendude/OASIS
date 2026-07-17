/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "Ssd1305Framebuffer.hpp"

#include <algorithm>
#include <stdexcept>

using OASIS::Display::Ssd1305Framebuffer;
using OASIS::Display::Ssd1305FramebufferConfig;
using OASIS::Display::Ssd1305ImageView;
using OASIS::Display::Ssd1305Rotation;

namespace
{
struct PixelFormat
{
  std::size_t bytes_per_pixel;
  std::size_t red_offset;
  std::size_t green_offset;
  std::size_t blue_offset;
  bool grayscale;
};

PixelFormat GetPixelFormat(std::string_view encoding)
{
  if (encoding == "mono8" || encoding == "8UC1")
    return {1, 0, 0, 0, true};
  if (encoding == "rgb8")
    return {3, 0, 1, 2, false};
  if (encoding == "bgr8")
    return {3, 2, 1, 0, false};
  if (encoding == "rgba8")
    return {4, 0, 1, 2, false};
  if (encoding == "bgra8")
    return {4, 2, 1, 0, false};

  throw std::invalid_argument("Unsupported SSD1305 image encoding '" + std::string(encoding) + "'");
}

std::size_t RotatedWidth(const Ssd1305ImageView& image, Ssd1305Rotation rotation)
{
  if (rotation == Ssd1305Rotation::Rotate90 || rotation == Ssd1305Rotation::Rotate270)
    return image.height;
  return image.width;
}

std::size_t RotatedHeight(const Ssd1305ImageView& image, Ssd1305Rotation rotation)
{
  if (rotation == Ssd1305Rotation::Rotate90 || rotation == Ssd1305Rotation::Rotate270)
    return image.width;
  return image.height;
}

std::pair<std::size_t, std::size_t> Unrotate(std::size_t x,
                                             std::size_t y,
                                             const Ssd1305ImageView& image,
                                             Ssd1305Rotation rotation)
{
  switch (rotation)
  {
    case Ssd1305Rotation::Rotate0:
      return {x, y};
    case Ssd1305Rotation::Rotate90:
      return {y, image.height - 1U - x};
    case Ssd1305Rotation::Rotate180:
      return {image.width - 1U - x, image.height - 1U - y};
    case Ssd1305Rotation::Rotate270:
      return {image.width - 1U - y, x};
  }

  throw std::invalid_argument("Unknown SSD1305 rotation");
}

std::uint8_t Luma(const std::uint8_t* pixel, const PixelFormat& format)
{
  if (format.grayscale)
    return pixel[0];

  const unsigned red = pixel[format.red_offset];
  const unsigned green = pixel[format.green_offset];
  const unsigned blue = pixel[format.blue_offset];

  // Integer BT.601 luma: (0.299 R + 0.587 G + 0.114 B) * 256
  return static_cast<std::uint8_t>((77U * red + 150U * green + 29U * blue) >> 8U);
}
} // namespace

Ssd1305Framebuffer::Ssd1305Framebuffer()
{
  Clear(false);
  MarkClean();
}

void Ssd1305Framebuffer::Clear(bool lit)
{
  const std::uint8_t value = lit ? 0xFF : 0x00;
  for (std::size_t page = 0; page < SSD1305_PAGE_COUNT; ++page)
  {
    const auto first = m_buffer.begin() + static_cast<std::ptrdiff_t>(page * SSD1305_DISPLAY_WIDTH);
    const auto last = first + static_cast<std::ptrdiff_t>(SSD1305_DISPLAY_WIDTH);
    if (!std::all_of(first, last, [value](std::uint8_t byte) { return byte == value; }))
      m_dirtyPages[page] = true;
    std::fill(first, last, value);
  }
}

void Ssd1305Framebuffer::Invert()
{
  for (std::uint8_t& byte : m_buffer)
    byte = static_cast<std::uint8_t>(~byte);
  m_dirtyPages.fill(true);
}

void Ssd1305Framebuffer::Update(const Ssd1305ImageView& image,
                                const Ssd1305FramebufferConfig& config)
{
  if (image.width == 0 || image.height == 0)
    throw std::invalid_argument("SSD1305 image dimensions must be positive");

  const PixelFormat format = GetPixelFormat(image.encoding);
  const std::size_t rotated_width = RotatedWidth(image, config.rotation);
  const std::size_t rotated_height = RotatedHeight(image, config.rotation);
  const bool exact_dimensions =
      rotated_width == SSD1305_DISPLAY_WIDTH && rotated_height == SSD1305_DISPLAY_HEIGHT;
  if (!exact_dimensions && config.reject_wrong_dimensions)
  {
    throw std::invalid_argument("SSD1305 source dimensions do not match the selected rotation");
  }
  if (!exact_dimensions && !config.clip_wrong_dimensions)
  {
    throw std::invalid_argument(
        "SSD1305 non-matching dimensions require clip_wrong_dimensions=true");
  }

  const std::size_t minimum_step = image.width * format.bytes_per_pixel;
  if (image.step < minimum_step)
    throw std::invalid_argument("SSD1305 image step is smaller than one encoded row");
  if (image.data.size() < image.step * (image.height - 1U) + minimum_step)
    throw std::invalid_argument("SSD1305 image data is shorter than dimensions require");

  Buffer next{};
  for (std::size_t y = 0; y < SSD1305_DISPLAY_HEIGHT; ++y)
  {
    for (std::size_t x = 0; x < SSD1305_DISPLAY_WIDTH; ++x)
    {
      const bool lit = SampleOutputPixel(image, config, x, y);
      if (!lit)
        continue;

      const std::size_t page = y / SSD1305_PAGE_HEIGHT;
      const std::size_t bit = y % SSD1305_PAGE_HEIGHT;
      next[page * SSD1305_DISPLAY_WIDTH + x] |= static_cast<std::uint8_t>(1U << bit);
    }
  }

  for (std::size_t page = 0; page < SSD1305_PAGE_COUNT; ++page)
  {
    const auto first = m_buffer.begin() + static_cast<std::ptrdiff_t>(page * SSD1305_DISPLAY_WIDTH);
    const auto last = first + static_cast<std::ptrdiff_t>(SSD1305_DISPLAY_WIDTH);
    const auto next_first =
        next.begin() + static_cast<std::ptrdiff_t>(page * SSD1305_DISPLAY_WIDTH);
    if (!std::equal(first, last, next_first))
      m_dirtyPages[page] = true;
  }

  m_buffer = next;
}

void Ssd1305Framebuffer::MarkClean()
{
  m_dirtyPages.fill(false);
}

bool Ssd1305Framebuffer::HasDirtyPages() const
{
  return std::any_of(m_dirtyPages.begin(), m_dirtyPages.end(), [](bool dirty) { return dirty; });
}

void Ssd1305Framebuffer::SetPixel(std::size_t x, std::size_t y, bool lit)
{
  const std::size_t page = y / SSD1305_PAGE_HEIGHT;
  const std::size_t bit = y % SSD1305_PAGE_HEIGHT;
  const std::uint8_t mask = static_cast<std::uint8_t>(1U << bit);
  std::uint8_t& value = m_buffer[page * SSD1305_DISPLAY_WIDTH + x];
  const std::uint8_t next =
      lit ? static_cast<std::uint8_t>(value | mask) : static_cast<std::uint8_t>(value & ~mask);
  if (value != next)
  {
    value = next;
    m_dirtyPages[page] = true;
  }
}

bool Ssd1305Framebuffer::SampleOutputPixel(const Ssd1305ImageView& image,
                                           const Ssd1305FramebufferConfig& config,
                                           std::size_t output_x,
                                           std::size_t output_y) const
{
  const PixelFormat format = GetPixelFormat(image.encoding);
  const std::size_t rotated_width = RotatedWidth(image, config.rotation);
  const std::size_t rotated_height = RotatedHeight(image, config.rotation);

  if (output_x >= rotated_width || output_y >= rotated_height)
    return false;

  const auto [source_x, source_y] = Unrotate(output_x, output_y, image, config.rotation);

  const std::size_t offset = source_y * image.step + source_x * format.bytes_per_pixel;
  const bool lit = Luma(image.data.data() + offset, format) > config.threshold;
  return config.invert_pixels ? !lit : lit;
}

std::vector<std::size_t> OASIS::Display::GetDirtyPageIndices(
    const Ssd1305Framebuffer::DirtyPages& pages)
{
  std::vector<std::size_t> dirty_pages;
  for (std::size_t page = 0; page < pages.size(); ++page)
  {
    if (pages[page])
      dirty_pages.push_back(page);
  }
  return dirty_pages;
}

Ssd1305Framebuffer::DirtyPages OASIS::Display::ComparePages(
    const Ssd1305Framebuffer::Buffer& confirmed, const Ssd1305Framebuffer::Buffer& desired)
{
  Ssd1305Framebuffer::DirtyPages dirty_pages{};
  for (std::size_t page = 0; page < SSD1305_PAGE_COUNT; ++page)
  {
    const auto confirmed_first =
        confirmed.begin() + static_cast<std::ptrdiff_t>(page * SSD1305_DISPLAY_WIDTH);
    const auto confirmed_last =
        confirmed_first + static_cast<std::ptrdiff_t>(SSD1305_DISPLAY_WIDTH);
    const auto desired_first =
        desired.begin() + static_cast<std::ptrdiff_t>(page * SSD1305_DISPLAY_WIDTH);
    dirty_pages[page] = !std::equal(confirmed_first, confirmed_last, desired_first);
  }
  return dirty_pages;
}
