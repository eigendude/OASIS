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

struct MapPointRenderInfo
{
  Eigen::Vector3f position = Eigen::Vector3f::Zero();
  bool tracked = false;
};

} // namespace SLAM
} // namespace OASIS
