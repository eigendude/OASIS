/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */
#pragma once

#include <stdint.h>

namespace OASIS
{
namespace LED
{

/*!
 * \brief Abstraction of a behavior that a group of LEDs can have
 */
class ILedBehavior
{
public:
  virtual ~ILedBehavior() = default;

  /*!
   * \brief Update the LEDs
   *
   * \param runtimeMs The time, in ms, since the LEDs were activated
   * \param progressRatio The completeness ratio of a task, if known, or 0.0f
   *                      otherwise
   */
  virtual void Update(uint64_t runtimeMs, float progressRatio) = 0;

  /*!
   * \brief Stop the LED behavior and leave LEDs in a final state
   */
  virtual void Shutdown() = 0;
};

} // namespace LED
} // namespace OASIS
