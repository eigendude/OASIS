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
using RGBColor = std::array<uint8_t, 3>;

constexpr std::array<RGBColor, 257> VIRIDIS_RGB = {{
    // clang-format off
    RGBColor{68, 1, 84},
    RGBColor{68, 2, 86},
    RGBColor{69, 4, 87},
    RGBColor{69, 5, 89},
    RGBColor{70, 7, 90},
    RGBColor{70, 8, 92},
    RGBColor{70, 10, 93},
    RGBColor{70, 11, 94},
    RGBColor{71, 13, 96},
    RGBColor{71, 14, 97},
    RGBColor{71, 16, 99},
    RGBColor{71, 17, 100},
    RGBColor{71, 19, 101},
    RGBColor{72, 20, 103},
    RGBColor{72, 22, 104},
    RGBColor{72, 23, 105},
    RGBColor{72, 24, 106},
    RGBColor{72, 26, 108},
    RGBColor{72, 27, 109},
    RGBColor{72, 28, 110},
    RGBColor{72, 29, 111},
    RGBColor{72, 31, 112},
    RGBColor{72, 32, 113},
    RGBColor{72, 33, 115},
    RGBColor{72, 35, 116},
    RGBColor{72, 36, 117},
    RGBColor{72, 37, 118},
    RGBColor{72, 38, 119},
    RGBColor{72, 40, 120},
    RGBColor{72, 41, 121},
    RGBColor{71, 42, 122},
    RGBColor{71, 44, 122},
    RGBColor{71, 45, 123},
    RGBColor{71, 46, 124},
    RGBColor{71, 47, 125},
    RGBColor{70, 48, 126},
    RGBColor{70, 50, 126},
    RGBColor{70, 51, 127},
    RGBColor{70, 52, 128},
    RGBColor{69, 53, 129},
    RGBColor{69, 55, 129},
    RGBColor{69, 56, 130},
    RGBColor{68, 57, 131},
    RGBColor{68, 58, 131},
    RGBColor{68, 59, 132},
    RGBColor{67, 61, 132},
    RGBColor{67, 62, 133},
    RGBColor{66, 63, 133},
    RGBColor{66, 64, 134},
    RGBColor{66, 65, 134},
    RGBColor{65, 66, 135},
    RGBColor{65, 68, 135},
    RGBColor{64, 69, 136},
    RGBColor{64, 70, 136},
    RGBColor{63, 71, 136},
    RGBColor{63, 72, 137},
    RGBColor{62, 73, 137},
    RGBColor{62, 74, 137},
    RGBColor{62, 76, 138},
    RGBColor{61, 77, 138},
    RGBColor{61, 78, 138},
    RGBColor{60, 79, 138},
    RGBColor{60, 80, 139},
    RGBColor{59, 81, 139},
    RGBColor{59, 82, 139},
    RGBColor{58, 83, 139},
    RGBColor{58, 84, 140},
    RGBColor{57, 85, 140},
    RGBColor{57, 86, 140},
    RGBColor{56, 88, 140},
    RGBColor{56, 89, 140},
    RGBColor{55, 90, 140},
    RGBColor{55, 91, 141},
    RGBColor{54, 92, 141},
    RGBColor{54, 93, 141},
    RGBColor{53, 94, 141},
    RGBColor{53, 95, 141},
    RGBColor{52, 96, 141},
    RGBColor{52, 97, 141},
    RGBColor{51, 98, 141},
    RGBColor{51, 99, 141},
    RGBColor{50, 100, 142},
    RGBColor{50, 101, 142},
    RGBColor{49, 102, 142},
    RGBColor{49, 103, 142},
    RGBColor{49, 104, 142},
    RGBColor{48, 105, 142},
    RGBColor{48, 106, 142},
    RGBColor{47, 107, 142},
    RGBColor{47, 108, 142},
    RGBColor{46, 109, 142},
    RGBColor{46, 110, 142},
    RGBColor{46, 111, 142},
    RGBColor{45, 112, 142},
    RGBColor{45, 113, 142},
    RGBColor{44, 113, 142},
    RGBColor{44, 114, 142},
    RGBColor{44, 115, 142},
    RGBColor{43, 116, 142},
    RGBColor{43, 117, 142},
    RGBColor{42, 118, 142},
    RGBColor{42, 119, 142},
    RGBColor{42, 120, 142},
    RGBColor{41, 121, 142},
    RGBColor{41, 122, 142},
    RGBColor{41, 123, 142},
    RGBColor{40, 124, 142},
    RGBColor{40, 125, 142},
    RGBColor{39, 126, 142},
    RGBColor{39, 127, 142},
    RGBColor{39, 128, 142},
    RGBColor{38, 129, 142},
    RGBColor{38, 130, 142},
    RGBColor{38, 130, 142},
    RGBColor{37, 131, 142},
    RGBColor{37, 132, 142},
    RGBColor{37, 133, 142},
    RGBColor{36, 134, 142},
    RGBColor{36, 135, 142},
    RGBColor{35, 136, 142},
    RGBColor{35, 137, 142},
    RGBColor{35, 138, 141},
    RGBColor{34, 139, 141},
    RGBColor{34, 140, 141},
    RGBColor{33, 141, 141},
    RGBColor{33, 142, 141},
    RGBColor{33, 143, 141},
    RGBColor{32, 144, 141},
    RGBColor{32, 145, 141},
    RGBColor{32, 146, 141},
    RGBColor{32, 146, 141},
    RGBColor{32, 147, 140},
    RGBColor{31, 148, 140},
    RGBColor{31, 149, 140},
    RGBColor{31, 149, 139},
    RGBColor{31, 150, 139},
    RGBColor{31, 151, 139},
    RGBColor{31, 152, 139},
    RGBColor{31, 153, 138},
    RGBColor{31, 154, 138},
    RGBColor{30, 155, 138},
    RGBColor{30, 156, 137},
    RGBColor{30, 157, 137},
    RGBColor{31, 158, 137},
    RGBColor{31, 159, 136},
    RGBColor{31, 160, 136},
    RGBColor{31, 161, 136},
    RGBColor{31, 161, 135},
    RGBColor{31, 162, 135},
    RGBColor{32, 163, 134},
    RGBColor{32, 164, 134},
    RGBColor{33, 165, 133},
    RGBColor{33, 166, 133},
    RGBColor{34, 167, 133},
    RGBColor{34, 168, 132},
    RGBColor{35, 169, 131},
    RGBColor{36, 170, 131},
    RGBColor{37, 171, 130},
    RGBColor{37, 172, 130},
    RGBColor{38, 173, 129},
    RGBColor{39, 173, 129},
    RGBColor{40, 174, 128},
    RGBColor{41, 175, 127},
    RGBColor{42, 176, 127},
    RGBColor{44, 177, 126},
    RGBColor{45, 178, 125},
    RGBColor{46, 179, 124},
    RGBColor{47, 180, 124},
    RGBColor{49, 181, 123},
    RGBColor{50, 182, 122},
    RGBColor{52, 182, 121},
    RGBColor{53, 183, 121},
    RGBColor{55, 184, 120},
    RGBColor{56, 185, 119},
    RGBColor{58, 186, 118},
    RGBColor{59, 187, 117},
    RGBColor{61, 188, 116},
    RGBColor{63, 188, 115},
    RGBColor{64, 189, 114},
    RGBColor{66, 190, 113},
    RGBColor{68, 191, 112},
    RGBColor{70, 192, 111},
    RGBColor{72, 193, 110},
    RGBColor{74, 193, 109},
    RGBColor{76, 194, 108},
    RGBColor{78, 195, 107},
    RGBColor{80, 196, 106},
    RGBColor{82, 197, 105},
    RGBColor{84, 197, 104},
    RGBColor{86, 198, 103},
    RGBColor{88, 199, 101},
    RGBColor{90, 200, 100},
    RGBColor{92, 200, 99},
    RGBColor{94, 201, 98},
    RGBColor{96, 202, 96},
    RGBColor{99, 203, 95},
    RGBColor{101, 203, 94},
    RGBColor{103, 204, 92},
    RGBColor{105, 205, 91},
    RGBColor{108, 205, 90},
    RGBColor{110, 206, 88},
    RGBColor{112, 207, 87},
    RGBColor{115, 208, 86},
    RGBColor{117, 208, 84},
    RGBColor{119, 209, 83},
    RGBColor{122, 209, 81},
    RGBColor{124, 210, 80},
    RGBColor{127, 211, 78},
    RGBColor{129, 211, 77},
    RGBColor{132, 212, 75},
    RGBColor{134, 213, 73},
    RGBColor{137, 213, 72},
    RGBColor{139, 214, 70},
    RGBColor{142, 214, 69},
    RGBColor{144, 215, 67},
    RGBColor{147, 215, 65},
    RGBColor{149, 216, 64},
    RGBColor{152, 216, 62},
    RGBColor{155, 217, 60},
    RGBColor{157, 217, 59},
    RGBColor{160, 218, 57},
    RGBColor{162, 218, 55},
    RGBColor{165, 219, 54},
    RGBColor{168, 219, 52},
    RGBColor{170, 220, 50},
    RGBColor{173, 220, 48},
    RGBColor{176, 221, 47},
    RGBColor{178, 221, 45},
    RGBColor{181, 222, 43},
    RGBColor{184, 222, 41},
    RGBColor{186, 222, 40},
    RGBColor{189, 223, 38},
    RGBColor{192, 223, 37},
    RGBColor{194, 223, 35},
    RGBColor{197, 224, 33},
    RGBColor{200, 224, 32},
    RGBColor{202, 225, 31},
    RGBColor{205, 225, 29},
    RGBColor{208, 225, 28},
    RGBColor{210, 226, 27},
    RGBColor{213, 226, 26},
    RGBColor{216, 226, 25},
    RGBColor{218, 227, 25},
    RGBColor{221, 227, 24},
    RGBColor{223, 227, 24},
    RGBColor{226, 228, 24},
    RGBColor{229, 228, 25},
    RGBColor{231, 228, 25},
    RGBColor{234, 229, 26},
    RGBColor{236, 229, 27},
    RGBColor{239, 229, 28},
    RGBColor{241, 229, 29},
    RGBColor{244, 230, 30},
    RGBColor{246, 230, 32},
    RGBColor{248, 230, 33},
    RGBColor{251, 231, 35},
    RGBColor{253, 231, 37},
    // clang-format on
}};

constexpr int ClampIndex(int index)
{
  return index < 0 ? 0
                   : (index >= static_cast<int>(VIRIDIS_RGB.size())
                          ? static_cast<int>(VIRIDIS_RGB.size()) - 1
                          : index);
}
} // namespace

const std::array<uint8_t, 3>& ViridisPaletteSampler::Lookup(int index)
{
  return VIRIDIS_RGB[ClampIndex(index)];
}

cv::Vec3b ViridisPaletteSampler::Sample(float value) const
{
  const float clamped = std::clamp(value, 0.0f, 1.0f);
  const float reversed = 1.0f - clamped;
  const float scaled = reversed * static_cast<float>(VIRIDIS_RGB.size() - 1);
  const int lowerIndex = static_cast<int>(std::floor(scaled));
  const int upperIndex = std::min(lowerIndex + 1, static_cast<int>(VIRIDIS_RGB.size() - 1));
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
