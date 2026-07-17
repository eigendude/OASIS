/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "display/ssd1305/Ssd1305Framebuffer.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <string_view>
#include <vector>

#include <gtest/gtest.h>

using OASIS::Display::ComparePages;
using OASIS::Display::SSD1305_DISPLAY_HEIGHT;
using OASIS::Display::SSD1305_DISPLAY_WIDTH;
using OASIS::Display::SSD1305_FRAMEBUFFER_SIZE;
using OASIS::Display::Ssd1305Framebuffer;
using OASIS::Display::Ssd1305FramebufferConfig;
using OASIS::Display::Ssd1305ImageView;
using OASIS::Display::Ssd1305Rotation;

namespace
{
Ssd1305ImageView MakeView(const std::vector<std::uint8_t>& image,
                          std::size_t width,
                          std::size_t height,
                          std::size_t step,
                          std::string_view encoding = "mono8")
{
  return {
      .data = image,
      .width = width,
      .height = height,
      .step = step,
      .encoding = encoding,
  };
}

std::size_t ByteIndex(std::size_t x, std::size_t y)
{
  return (y / 8U) * SSD1305_DISPLAY_WIDTH + x;
}

std::uint8_t BitMask(std::size_t y)
{
  return static_cast<std::uint8_t>(1U << (y % 8U));
}

Ssd1305FramebufferConfig DefaultConfig()
{
  return {
      .rotation = Ssd1305Rotation::Rotate0,
      .threshold = 127,
      .invert_pixels = false,
      .reject_wrong_dimensions = true,
      .clip_wrong_dimensions = false,
  };
}

std::pair<std::size_t, std::size_t> SourceForOutput(std::size_t x,
                                                    std::size_t y,
                                                    std::size_t source_width,
                                                    std::size_t source_height,
                                                    Ssd1305Rotation rotation)
{
  switch (rotation)
  {
    case Ssd1305Rotation::Rotate0:
      return {x, y};
    case Ssd1305Rotation::Rotate90:
      return {y, source_height - 1U - x};
    case Ssd1305Rotation::Rotate180:
      return {source_width - 1U - x, source_height - 1U - y};
    case Ssd1305Rotation::Rotate270:
      return {source_width - 1U - y, x};
  }
  return {0, 0};
}
} // namespace

static_assert(SSD1305_FRAMEBUFFER_SIZE == 512);

TEST(Ssd1305Framebuffer, PacksAllOffAndAllOnImages)
{
  std::vector<std::uint8_t> image(SSD1305_DISPLAY_WIDTH * SSD1305_DISPLAY_HEIGHT, 0);
  Ssd1305Framebuffer framebuffer;
  framebuffer.Update(
      MakeView(image, SSD1305_DISPLAY_WIDTH, SSD1305_DISPLAY_HEIGHT, SSD1305_DISPLAY_WIDTH),
      DefaultConfig());
  EXPECT_TRUE(std::all_of(framebuffer.Data().begin(), framebuffer.Data().end(),
                          [](std::uint8_t byte) { return byte == 0; }));

  std::fill(image.begin(), image.end(), 255);
  framebuffer.Update(
      MakeView(image, SSD1305_DISPLAY_WIDTH, SSD1305_DISPLAY_HEIGHT, SSD1305_DISPLAY_WIDTH),
      DefaultConfig());
  EXPECT_TRUE(std::all_of(framebuffer.Data().begin(), framebuffer.Data().end(),
                          [](std::uint8_t byte) { return byte == 0xFF; }));
}

TEST(Ssd1305Framebuffer, PacksCornersAndPageBoundaries)
{
  std::vector<std::uint8_t> image(SSD1305_DISPLAY_WIDTH * SSD1305_DISPLAY_HEIGHT, 0);
  const std::array<std::pair<std::size_t, std::size_t>, 11> pixels{{
      {0, 0},
      {127, 0},
      {0, 31},
      {127, 31},
      {3, 7},
      {4, 8},
      {5, 15},
      {6, 16},
      {7, 23},
      {8, 24},
      {9, 31},
  }};
  for (const auto& [x, y] : pixels)
    image[y * SSD1305_DISPLAY_WIDTH + x] = 255;

  Ssd1305Framebuffer framebuffer;
  framebuffer.Update(
      MakeView(image, SSD1305_DISPLAY_WIDTH, SSD1305_DISPLAY_HEIGHT, SSD1305_DISPLAY_WIDTH),
      DefaultConfig());
  for (const auto& [x, y] : pixels)
    EXPECT_NE(framebuffer.Data()[ByteIndex(x, y)] & BitMask(y), 0);
}

TEST(Ssd1305Framebuffer, AcceptsPaddedRows)
{
  constexpr std::size_t step = SSD1305_DISPLAY_WIDTH + 7U;
  std::vector<std::uint8_t> image(step * SSD1305_DISPLAY_HEIGHT, 0);
  image[step + 2U] = 255;
  Ssd1305Framebuffer framebuffer;
  framebuffer.Update(MakeView(image, SSD1305_DISPLAY_WIDTH, SSD1305_DISPLAY_HEIGHT, step),
                     DefaultConfig());
  EXPECT_EQ(framebuffer.Data()[2], 0b00000010);
}

TEST(Ssd1305Framebuffer, ValidatesStepDataLengthEncodingAndDimensions)
{
  std::vector<std::uint8_t> image(SSD1305_DISPLAY_WIDTH * SSD1305_DISPLAY_HEIGHT, 0);
  Ssd1305Framebuffer framebuffer;
  EXPECT_THROW(framebuffer.Update(MakeView(image, SSD1305_DISPLAY_WIDTH, SSD1305_DISPLAY_HEIGHT,
                                           SSD1305_DISPLAY_WIDTH - 1U),
                                  DefaultConfig()),
               std::invalid_argument);

  image.pop_back();
  EXPECT_THROW(framebuffer.Update(MakeView(image, SSD1305_DISPLAY_WIDTH, SSD1305_DISPLAY_HEIGHT,
                                           SSD1305_DISPLAY_WIDTH),
                                  DefaultConfig()),
               std::invalid_argument);

  std::vector<std::uint8_t> wrong_size(64U * 32U, 0);
  EXPECT_THROW(framebuffer.Update(MakeView(wrong_size, 64, 32, 64), DefaultConfig()),
               std::invalid_argument);

  std::vector<std::uint8_t> exact(SSD1305_DISPLAY_WIDTH * SSD1305_DISPLAY_HEIGHT, 0);
  EXPECT_THROW(framebuffer.Update(MakeView(exact, SSD1305_DISPLAY_WIDTH, SSD1305_DISPLAY_HEIGHT,
                                           SSD1305_DISPLAY_WIDTH, "yuv422"),
                                  DefaultConfig()),
               std::invalid_argument);
}

TEST(Ssd1305Framebuffer, ClipsWithoutScalingWhenExplicitlyEnabled)
{
  std::vector<std::uint8_t> image(64U * 16U, 255);
  Ssd1305FramebufferConfig config = DefaultConfig();
  config.reject_wrong_dimensions = false;
  config.clip_wrong_dimensions = true;
  Ssd1305Framebuffer framebuffer;
  framebuffer.Update(MakeView(image, 64, 16, 64), config);
  EXPECT_EQ(framebuffer.Data()[ByteIndex(63, 15)], 0xFF);
  EXPECT_EQ(framebuffer.Data()[ByteIndex(64, 15)], 0);
  EXPECT_EQ(framebuffer.Data()[ByteIndex(0, 16)], 0);
}

TEST(Ssd1305Framebuffer, ThresholdIsStrictlyGreaterThanConfiguredValue)
{
  std::vector<std::uint8_t> image(SSD1305_DISPLAY_WIDTH * SSD1305_DISPLAY_HEIGHT, 0);
  image[0] = 126;
  image[1] = 127;
  image[2] = 128;
  Ssd1305Framebuffer framebuffer;
  framebuffer.Update(
      MakeView(image, SSD1305_DISPLAY_WIDTH, SSD1305_DISPLAY_HEIGHT, SSD1305_DISPLAY_WIDTH),
      DefaultConfig());
  EXPECT_EQ(framebuffer.Data()[0], 0);
  EXPECT_EQ(framebuffer.Data()[1], 0);
  EXPECT_EQ(framebuffer.Data()[2], 1);
}

TEST(Ssd1305Framebuffer, InvertsCurrentFramebufferAndMarksAllPagesDirty)
{
  std::vector<std::uint8_t> image(SSD1305_DISPLAY_WIDTH * SSD1305_DISPLAY_HEIGHT, 0);
  image[0] = 255;

  Ssd1305Framebuffer framebuffer;
  framebuffer.Update(
      MakeView(image, SSD1305_DISPLAY_WIDTH, SSD1305_DISPLAY_HEIGHT, SSD1305_DISPLAY_WIDTH),
      DefaultConfig());
  framebuffer.MarkClean();

  framebuffer.Invert();
  EXPECT_EQ(framebuffer.Data()[0], 0xFE);
  for (bool dirty : framebuffer.Dirty())
    EXPECT_TRUE(dirty);
}

TEST(Ssd1305Framebuffer, InvertingTwiceRestoresOriginalFramebuffer)
{
  std::vector<std::uint8_t> image(SSD1305_DISPLAY_WIDTH * SSD1305_DISPLAY_HEIGHT, 0);
  image[0] = 255;
  image[15U * SSD1305_DISPLAY_WIDTH + 12U] = 255;

  Ssd1305Framebuffer framebuffer;
  framebuffer.Update(
      MakeView(image, SSD1305_DISPLAY_WIDTH, SSD1305_DISPLAY_HEIGHT, SSD1305_DISPLAY_WIDTH),
      DefaultConfig());
  const auto original = framebuffer.Data();

  framebuffer.Invert();
  framebuffer.Invert();

  EXPECT_EQ(framebuffer.Data(), original);
}

class Ssd1305ColorEncodingTest : public testing::TestWithParam<std::string_view>
{
};

TEST_P(Ssd1305ColorEncodingTest, ConvertsColorPixelUsingBt601Luma)
{
  const std::string_view encoding = GetParam();
  const std::size_t bytes_per_pixel = encoding.ends_with("a8") ? 4U : 3U;
  std::vector<std::uint8_t> image(SSD1305_DISPLAY_WIDTH * SSD1305_DISPLAY_HEIGHT * bytes_per_pixel,
                                  0);
  image[1] = 255;
  Ssd1305FramebufferConfig config = DefaultConfig();
  config.threshold = 148;
  Ssd1305Framebuffer framebuffer;
  framebuffer.Update(MakeView(image, SSD1305_DISPLAY_WIDTH, SSD1305_DISPLAY_HEIGHT,
                              SSD1305_DISPLAY_WIDTH * bytes_per_pixel, encoding),
                     config);
  EXPECT_EQ(framebuffer.Data()[0], 1);
}

INSTANTIATE_TEST_SUITE_P(ColorEncodings,
                         Ssd1305ColorEncodingTest,
                         testing::Values("rgb8", "bgr8", "rgba8", "bgra8"));

class Ssd1305RotationTest : public testing::TestWithParam<Ssd1305Rotation>
{
};

TEST_P(Ssd1305RotationTest, MapsAllFourCornersExactly)
{
  const Ssd1305Rotation rotation = GetParam();
  const bool quarter_turn =
      rotation == Ssd1305Rotation::Rotate90 || rotation == Ssd1305Rotation::Rotate270;
  const std::size_t source_width = quarter_turn ? SSD1305_DISPLAY_HEIGHT : SSD1305_DISPLAY_WIDTH;
  const std::size_t source_height = quarter_turn ? SSD1305_DISPLAY_WIDTH : SSD1305_DISPLAY_HEIGHT;
  std::vector<std::uint8_t> image(source_width * source_height, 0);
  const std::array<std::pair<std::size_t, std::size_t>, 4> corners{{
      {0, 0},
      {SSD1305_DISPLAY_WIDTH - 1U, 0},
      {0, SSD1305_DISPLAY_HEIGHT - 1U},
      {SSD1305_DISPLAY_WIDTH - 1U, SSD1305_DISPLAY_HEIGHT - 1U},
  }};
  for (const auto& [x, y] : corners)
  {
    const auto [source_x, source_y] = SourceForOutput(x, y, source_width, source_height, rotation);
    image[source_y * source_width + source_x] = 255;
  }

  Ssd1305FramebufferConfig config = DefaultConfig();
  config.rotation = rotation;
  Ssd1305Framebuffer framebuffer;
  framebuffer.Update(MakeView(image, source_width, source_height, source_width), config);
  for (const auto& [x, y] : corners)
    EXPECT_NE(framebuffer.Data()[ByteIndex(x, y)] & BitMask(y), 0);
}

INSTANTIATE_TEST_SUITE_P(AllRotations,
                         Ssd1305RotationTest,
                         testing::Values(Ssd1305Rotation::Rotate0,
                                         Ssd1305Rotation::Rotate90,
                                         Ssd1305Rotation::Rotate180,
                                         Ssd1305Rotation::Rotate270));

TEST(Ssd1305Framebuffer, ComparesDesiredPagesAgainstConfirmedHardware)
{
  Ssd1305Framebuffer::Buffer confirmed{};
  Ssd1305Framebuffer::Buffer desired{};
  EXPECT_EQ(ComparePages(confirmed, desired), (Ssd1305Framebuffer::DirtyPages{}));

  desired[0] = 1;
  desired[3U * SSD1305_DISPLAY_WIDTH + 127U] = 1;
  const auto dirty = ComparePages(confirmed, desired);
  EXPECT_TRUE(dirty[0]);
  EXPECT_FALSE(dirty[1]);
  EXPECT_FALSE(dirty[2]);
  EXPECT_TRUE(dirty[3]);
}

TEST(Ssd1305Framebuffer, ClearMarksOnlyChangedPages)
{
  Ssd1305Framebuffer framebuffer;
  framebuffer.MarkClean();
  framebuffer.Clear(false);
  EXPECT_FALSE(framebuffer.HasDirtyPages());

  std::vector<std::uint8_t> image(SSD1305_DISPLAY_WIDTH * SSD1305_DISPLAY_HEIGHT, 0);
  image[8U * SSD1305_DISPLAY_WIDTH] = 255;
  framebuffer.Update(
      MakeView(image, SSD1305_DISPLAY_WIDTH, SSD1305_DISPLAY_HEIGHT, SSD1305_DISPLAY_WIDTH),
      DefaultConfig());
  framebuffer.MarkClean();
  framebuffer.Clear(false);
  EXPECT_FALSE(framebuffer.Dirty()[0]);
  EXPECT_TRUE(framebuffer.Dirty()[1]);
  EXPECT_FALSE(framebuffer.Dirty()[2]);
  EXPECT_FALSE(framebuffer.Dirty()[3]);
}
