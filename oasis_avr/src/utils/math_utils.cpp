/*
 *  Copyright (C) 2022-2024 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "math_utils.hpp"

using namespace OASIS;

float MathUtils::Clamp(float value, float min, float max)
{
  if (value < min)
    value = min;
  if (value > max)
    value = max;
  return value;
}
