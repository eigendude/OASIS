/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <opencv2/core.hpp>

namespace OASIS
{
namespace SLAM
{

class ViridisPaletteSampler
{
public:
  ViridisPaletteSampler() = default;

  cv::Vec3b Sample(float value) const;

private:
  static const cv::Vec3b& Lookup(int index);
};

} // namespace SLAM
} // namespace OASIS
