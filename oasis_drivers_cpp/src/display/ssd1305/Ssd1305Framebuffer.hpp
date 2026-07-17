/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>
#include <vector>

namespace OASIS::Display
{
constexpr std::size_t SSD1305_DISPLAY_WIDTH = 128;
constexpr std::size_t SSD1305_DISPLAY_HEIGHT = 32;
constexpr std::size_t SSD1305_PAGE_HEIGHT = 8;
constexpr std::size_t SSD1305_PAGE_COUNT = SSD1305_DISPLAY_HEIGHT / SSD1305_PAGE_HEIGHT;
constexpr std::size_t SSD1305_FRAMEBUFFER_SIZE = SSD1305_DISPLAY_WIDTH * SSD1305_PAGE_COUNT;

/** \brief Clockwise image rotation applied before packing SSD1305 page bytes */
enum class Ssd1305Rotation
{
  /** \brief No rotation; output pixel (x, y) samples source pixel (x, y) */
  Rotate0,

  /** \brief Rotate source 90 degrees clockwise into the display frame */
  Rotate90,

  /** \brief Rotate source 180 degrees into the display frame */
  Rotate180,

  /** \brief Rotate source 270 degrees clockwise into the display frame */
  Rotate270,
};

/** \brief Plain image view used by the ROS-free SSD1305 framebuffer converter */
struct Ssd1305ImageView
{
  /** \brief Image bytes in row-major order, using the named encoding */
  std::span<const std::uint8_t> data;

  /** \brief Source image width in pixels; must be greater than zero */
  std::size_t width;

  /** \brief Source image height in pixels; must be greater than zero */
  std::size_t height;

  /** \brief Source row stride in bytes; must cover one encoded row */
  std::size_t step;

  /** \brief ROS-style encoding name, such as mono8, rgb8, bgr8, or rgba8 */
  std::string_view encoding;
};

/** \brief Conversion policy for image pixels packed into SSD1305 GDDRAM bytes */
struct Ssd1305FramebufferConfig
{
  /** \brief Clockwise source rotation before mapping pixels to 128 x 32 */
  Ssd1305Rotation rotation;

  /** \brief Luma threshold in [0, 255]; pixels above it become lit */
  std::uint8_t threshold;

  /** \brief Whether black/white conversion is inverted after thresholding */
  bool invert_pixels;

  /** \brief Reject sources whose rotated dimensions are not exactly 128 x 32 */
  bool reject_wrong_dimensions;

  /** \brief Clip non-matching sources at the output origin instead of scaling */
  bool clip_wrong_dimensions;
};

/** \brief Packed 128 x 32 SSD1305 framebuffer with dirty-page tracking */
class Ssd1305Framebuffer
{
public:
  using Buffer = std::array<std::uint8_t, SSD1305_FRAMEBUFFER_SIZE>;
  using DirtyPages = std::array<bool, SSD1305_PAGE_COUNT>;

  Ssd1305Framebuffer();

  void Clear(bool lit = false);
  void Invert();
  void Update(const Ssd1305ImageView& image, const Ssd1305FramebufferConfig& config);
  void MarkClean();

  const Buffer& Data() const { return m_buffer; }
  const DirtyPages& Dirty() const { return m_dirtyPages; }
  bool HasDirtyPages() const;

private:
  void SetPixel(std::size_t x, std::size_t y, bool lit);
  bool SampleOutputPixel(const Ssd1305ImageView& image,
                         const Ssd1305FramebufferConfig& config,
                         std::size_t output_x,
                         std::size_t output_y) const;

  Buffer m_buffer{};
  DirtyPages m_dirtyPages{};
};

std::vector<std::size_t> GetDirtyPageIndices(const Ssd1305Framebuffer::DirtyPages& pages);

Ssd1305Framebuffer::DirtyPages ComparePages(const Ssd1305Framebuffer::Buffer& confirmed,
                                            const Ssd1305Framebuffer::Buffer& desired);

} // namespace OASIS::Display
