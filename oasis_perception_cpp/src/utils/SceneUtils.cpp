/*
 *  Copyright (C) 2020-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "SceneUtils.h"

#include <algorithm>
#include <cmath>

using namespace OASIS::UTILS;

unsigned int SceneUtils::GetStride(unsigned int width)
{
  return width;
}

unsigned int SceneUtils::GetImageBufferLength(unsigned int width, unsigned int height)
{
  return GetStride(width) * height;
}

float SceneUtils::CalcSceneScore(float currentMafd, float previousMafd)
{
  const float diff = std::abs(currentMafd - previousMafd);

  return ClipValue(std::min(currentMafd, diff) / 100.0f, 0.0f, 1.0f);
}

float SceneUtils::CalcSceneMAFD(const uint8_t* previousFrame,
                                const uint8_t* currentFrame,
                                unsigned int width,
                                unsigned int height)
{
  // Calculate SAD
  const uint8_t* const src1 = previousFrame;
  const ptrdiff_t stride1 = GetStride(width);
  const uint8_t* const src2 = currentFrame;
  const ptrdiff_t stride2 = GetStride(width);
  const uint64_t sad = CalcSceneSAD(src1, stride1, src2, stride2, width, height);

  // Count pixels
  const uint64_t count = width * height;

  // Calculate mean absolute frame difference
  const float mafd = static_cast<float>(sad) / static_cast<float>(count);

  return mafd;
}

uint64_t SceneUtils::CalcSceneSAD(const uint8_t* src1,
                                  ptrdiff_t stride1,
                                  const uint8_t* src2,
                                  ptrdiff_t stride2,
                                  ptrdiff_t width,
                                  ptrdiff_t height)
{
  uint64_t sad = 0;

  for (unsigned y = 0; y < height; y++)
  {
    for (unsigned x = 0; x < width; x++)
      sad += std::abs(src1[x] - src2[x]);
    src1 += stride1;
    src2 += stride2;
  }

  return sad;
}

float SceneUtils::ClipValue(float value, float min, float max)
{
  if (value < min)
    value = min;
  else if (value > max)
    value = max;

  return value;
}
