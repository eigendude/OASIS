/*
 *  Copyright (C) 2020-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

namespace OASIS
{
namespace UTILS
{
class MathUtils
{
public:
  /*!
   * \brief Calculate the geometric mean of two dimensions
   *
   * \param width An image width, in pixels
   * \param height An image height, in pixels
   *
   * \return The geometric mean, the square root of the product
   */
  static double GeometricMean(unsigned int width, unsigned int height);
};
} // namespace UTILS
} // namespace OASIS
