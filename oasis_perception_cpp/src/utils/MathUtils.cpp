/*
 *  Copyright (C) 2020-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "MathUtils.h"

#include <cmath>

using namespace OASIS;
using namespace UTILS;

double MathUtils::GeometricMean(unsigned int width, unsigned int height)
{
  double product = static_cast<double>(width) * static_cast<double>(height);

  return std::sqrt(product);
}
