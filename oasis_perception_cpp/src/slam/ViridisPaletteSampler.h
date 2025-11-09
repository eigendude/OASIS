/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <Eigen/Core>

namespace OASIS
{
namespace SLAM
{

class ViridisPaletteSampler
{
public:
  static Eigen::Vector3f Sample(float t);
};

} // namespace SLAM
} // namespace OASIS
