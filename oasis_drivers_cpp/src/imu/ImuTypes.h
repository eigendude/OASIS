/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <array>

namespace OASIS::IMU
{
using Vec3 = std::array<double, 3>;
using Mat3 = std::array<std::array<double, 3>, 3>;
} // namespace OASIS::IMU
