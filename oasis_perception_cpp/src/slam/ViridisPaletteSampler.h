/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <array>
#include <cstdint>

#include <opencv2/core.hpp>

namespace OASIS
{
namespace SLAM
{

class ViridisPaletteSampler
{
public:
  ViridisPaletteSampler() = default;

  //! Sample the Viridis colormap returning a BGR triplet (B, G, R)
  cv::Vec3b Sample(float value) const;

private:
  //! Lookup an RGB entry from the precomputed Viridis table
  static const std::array<uint8_t, 3>& Lookup(int index);
};

} // namespace SLAM
} // namespace OASIS
