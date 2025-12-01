/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "ViridisPaletteSampler.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

using namespace OASIS;
using namespace SLAM;

namespace
{
using BGRColor = std::array<uint8_t, 3>;

constexpr std::array<BGRColor, 257> VIRIDIS_BGR = {{
    // clang-format off
    BGRColor{84, 1, 68},
    BGRColor{86, 2, 68},
    BGRColor{87, 4, 69},
    BGRColor{89, 5, 69},
    BGRColor{90, 7, 70},
    BGRColor{92, 8, 70},
    BGRColor{93, 10, 70},
    BGRColor{94, 11, 70},
    BGRColor{96, 13, 71},
    BGRColor{97, 14, 71},
    BGRColor{99, 16, 71},
    BGRColor{100, 17, 71},
    BGRColor{101, 19, 71},
    BGRColor{103, 20, 72},
    BGRColor{104, 22, 72},
    BGRColor{105, 23, 72},
    BGRColor{106, 24, 72},
    BGRColor{108, 26, 72},
    BGRColor{109, 27, 72},
    BGRColor{110, 28, 72},
    BGRColor{111, 29, 72},
    BGRColor{112, 31, 72},
    BGRColor{113, 32, 72},
    BGRColor{115, 33, 72},
    BGRColor{116, 35, 72},
    BGRColor{117, 36, 72},
    BGRColor{118, 37, 72},
    BGRColor{119, 38, 72},
    BGRColor{120, 40, 72},
    BGRColor{121, 41, 72},
    BGRColor{122, 42, 71},
    BGRColor{122, 44, 71},
    BGRColor{123, 45, 71},
    BGRColor{124, 46, 71},
    BGRColor{125, 47, 71},
    BGRColor{126, 48, 70},
    BGRColor{126, 50, 70},
    BGRColor{127, 51, 70},
    BGRColor{128, 52, 70},
    BGRColor{129, 53, 69},
    BGRColor{129, 55, 69},
    BGRColor{130, 56, 69},
    BGRColor{131, 57, 68},
    BGRColor{131, 58, 68},
    BGRColor{132, 59, 68},
    BGRColor{132, 61, 67},
    BGRColor{133, 62, 67},
    BGRColor{133, 63, 66},
    BGRColor{134, 64, 66},
    BGRColor{134, 65, 66},
    BGRColor{135, 66, 65},
    BGRColor{135, 68, 65},
    BGRColor{136, 69, 64},
    BGRColor{136, 70, 64},
    BGRColor{136, 71, 63},
    BGRColor{137, 72, 63},
    BGRColor{137, 73, 62},
    BGRColor{137, 74, 62},
    BGRColor{138, 76, 62},
    BGRColor{138, 77, 61},
    BGRColor{138, 78, 61},
    BGRColor{138, 79, 60},
    BGRColor{139, 80, 60},
    BGRColor{139, 81, 59},
    BGRColor{139, 82, 59},
    BGRColor{139, 83, 58},
    BGRColor{140, 84, 58},
    BGRColor{140, 85, 57},
    BGRColor{140, 86, 57},
    BGRColor{140, 88, 56},
    BGRColor{140, 89, 56},
    BGRColor{140, 90, 55},
    BGRColor{141, 91, 55},
    BGRColor{141, 92, 54},
    BGRColor{141, 93, 54},
    BGRColor{141, 94, 53},
    BGRColor{141, 95, 53},
    BGRColor{141, 96, 52},
    BGRColor{141, 97, 52},
    BGRColor{141, 98, 51},
    BGRColor{141, 99, 51},
    BGRColor{142, 100, 50},
    BGRColor{142, 101, 50},
    BGRColor{142, 102, 49},
    BGRColor{142, 103, 49},
    BGRColor{142, 104, 49},
    BGRColor{142, 105, 48},
    BGRColor{142, 106, 48},
    BGRColor{142, 107, 47},
    BGRColor{142, 108, 47},
    BGRColor{142, 109, 46},
    BGRColor{142, 110, 46},
    BGRColor{142, 111, 46},
    BGRColor{142, 112, 45},
    BGRColor{142, 113, 45},
    BGRColor{142, 113, 44},
    BGRColor{142, 114, 44},
    BGRColor{142, 115, 44},
    BGRColor{142, 116, 43},
    BGRColor{142, 117, 43},
    BGRColor{142, 118, 42},
    BGRColor{142, 119, 42},
    BGRColor{142, 120, 42},
    BGRColor{142, 121, 41},
    BGRColor{142, 122, 41},
    BGRColor{142, 123, 41},
    BGRColor{142, 124, 40},
    BGRColor{142, 125, 40},
    BGRColor{142, 126, 39},
    BGRColor{142, 127, 39},
    BGRColor{142, 128, 39},
    BGRColor{142, 129, 38},
    BGRColor{142, 130, 38},
    BGRColor{142, 130, 38},
    BGRColor{142, 131, 37},
    BGRColor{142, 132, 37},
    BGRColor{142, 133, 37},
    BGRColor{142, 134, 36},
    BGRColor{142, 135, 36},
    BGRColor{142, 136, 35},
    BGRColor{142, 137, 35},
    BGRColor{141, 138, 35},
    BGRColor{141, 139, 34},
    BGRColor{141, 140, 34},
    BGRColor{141, 141, 33},
    BGRColor{141, 142, 33},
    BGRColor{141, 143, 33},
    BGRColor{141, 144, 32},
    BGRColor{141, 145, 32},
    BGRColor{141, 146, 32},
    BGRColor{141, 146, 32},
    BGRColor{140, 147, 32},
    BGRColor{140, 148, 31},
    BGRColor{140, 149, 31},
    BGRColor{139, 149, 31},
    BGRColor{139, 150, 31},
    BGRColor{139, 151, 31},
    BGRColor{139, 152, 31},
    BGRColor{138, 153, 31},
    BGRColor{138, 154, 31},
    BGRColor{138, 155, 30},
    BGRColor{137, 156, 30},
    BGRColor{137, 157, 30},
    BGRColor{137, 158, 31},
    BGRColor{136, 159, 31},
    BGRColor{136, 160, 31},
    BGRColor{136, 161, 31},
    BGRColor{135, 161, 31},
    BGRColor{135, 162, 31},
    BGRColor{134, 163, 32},
    BGRColor{134, 164, 32},
    BGRColor{133, 165, 33},
    BGRColor{133, 166, 33},
    BGRColor{133, 167, 34},
    BGRColor{132, 168, 34},
    BGRColor{131, 169, 35},
    BGRColor{131, 170, 36},
    BGRColor{130, 171, 37},
    BGRColor{130, 172, 37},
    BGRColor{129, 173, 38},
    BGRColor{129, 173, 39},
    BGRColor{128, 174, 40},
    BGRColor{127, 175, 41},
    BGRColor{127, 176, 42},
    BGRColor{126, 177, 44},
    BGRColor{125, 178, 45},
    BGRColor{124, 179, 46},
    BGRColor{124, 180, 47},
    BGRColor{123, 181, 49},
    BGRColor{122, 182, 50},
    BGRColor{121, 182, 52},
    BGRColor{121, 183, 53},
    BGRColor{120, 184, 55},
    BGRColor{119, 185, 56},
    BGRColor{118, 186, 58},
    BGRColor{117, 187, 59},
    BGRColor{116, 188, 61},
    BGRColor{115, 188, 63},
    BGRColor{114, 189, 64},
    BGRColor{113, 190, 66},
    BGRColor{112, 191, 68},
    BGRColor{111, 192, 70},
    BGRColor{110, 193, 72},
    BGRColor{109, 193, 74},
    BGRColor{108, 194, 76},
    BGRColor{107, 195, 78},
    BGRColor{106, 196, 80},
    BGRColor{105, 197, 82},
    BGRColor{104, 197, 84},
    BGRColor{103, 198, 86},
    BGRColor{101, 199, 88},
    BGRColor{100, 200, 90},
    BGRColor{99, 200, 92},
    BGRColor{98, 201, 94},
    BGRColor{96, 202, 96},
    BGRColor{95, 203, 99},
    BGRColor{94, 203, 101},
    BGRColor{92, 204, 103},
    BGRColor{91, 205, 105},
    BGRColor{90, 205, 108},
    BGRColor{88, 206, 110},
    BGRColor{87, 207, 112},
    BGRColor{86, 208, 115},
    BGRColor{84, 208, 117},
    BGRColor{83, 209, 119},
    BGRColor{81, 209, 122},
    BGRColor{80, 210, 124},
    BGRColor{78, 211, 127},
    BGRColor{77, 211, 129},
    BGRColor{75, 212, 132},
    BGRColor{73, 213, 134},
    BGRColor{72, 213, 137},
    BGRColor{70, 214, 139},
    BGRColor{69, 214, 142},
    BGRColor{67, 215, 144},
    BGRColor{65, 215, 147},
    BGRColor{64, 216, 149},
    BGRColor{62, 216, 152},
    BGRColor{60, 217, 155},
    BGRColor{59, 217, 157},
    BGRColor{57, 218, 160},
    BGRColor{55, 218, 162},
    BGRColor{54, 219, 165},
    BGRColor{52, 219, 168},
    BGRColor{50, 220, 170},
    BGRColor{48, 220, 173},
    BGRColor{47, 221, 176},
    BGRColor{45, 221, 178},
    BGRColor{43, 222, 181},
    BGRColor{41, 222, 184},
    BGRColor{40, 222, 186},
    BGRColor{38, 223, 189},
    BGRColor{37, 223, 192},
    BGRColor{35, 223, 194},
    BGRColor{33, 224, 197},
    BGRColor{32, 224, 200},
    BGRColor{31, 225, 202},
    BGRColor{29, 225, 205},
    BGRColor{28, 225, 208},
    BGRColor{27, 226, 210},
    BGRColor{26, 226, 213},
    BGRColor{25, 226, 216},
    BGRColor{25, 227, 218},
    BGRColor{24, 227, 221},
    BGRColor{24, 227, 223},
    BGRColor{24, 228, 226},
    BGRColor{25, 228, 229},
    BGRColor{25, 228, 231},
    BGRColor{26, 229, 234},
    BGRColor{27, 229, 236},
    BGRColor{28, 229, 239},
    BGRColor{29, 229, 241},
    BGRColor{30, 230, 244},
    BGRColor{32, 230, 246},
    BGRColor{33, 230, 248},
    BGRColor{35, 231, 251},
    BGRColor{37, 231, 253},
    // clang-format on
}};

constexpr int ClampIndex(int index)
{
  return index < 0 ? 0
                   : (index >= static_cast<int>(VIRIDIS_BGR.size())
                          ? static_cast<int>(VIRIDIS_BGR.size()) - 1
                          : index);
}
} // namespace

const std::array<uint8_t, 3>& ViridisPaletteSampler::Lookup(int index)
{
  return VIRIDIS_BGR[ClampIndex(index)];
}

cv::Vec3b ViridisPaletteSampler::Sample(float value) const
{
  const float clamped = std::clamp(value, 0.0f, 1.0f);
  const float reversed = 1.0f - clamped;
  const float scaled = reversed * static_cast<float>(VIRIDIS_BGR.size() - 1);
  const int lowerIndex = static_cast<int>(std::floor(scaled));
  const int upperIndex = std::min(lowerIndex + 1, static_cast<int>(VIRIDIS_BGR.size() - 1));
  const float t = scaled - static_cast<float>(lowerIndex);

  if (upperIndex == lowerIndex)
  {
    const auto& color = Lookup(lowerIndex);
    return cv::Vec3b(color[0], color[1], color[2]);
  }

  const auto& lowerColor = Lookup(lowerIndex);
  const auto& upperColor = Lookup(upperIndex);
  cv::Vec3b result;
  for (int i = 0; i < 3; ++i)
  {
    const float channel =
        static_cast<float>(lowerColor[i]) + t * static_cast<float>(upperColor[i] - lowerColor[i]);
    result[i] = static_cast<uint8_t>(std::lround(channel));
  }

  return result;
}
