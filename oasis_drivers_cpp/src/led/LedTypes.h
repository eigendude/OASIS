/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */
#pragma once

#include <memory>
#include <vector>

namespace OASIS
{
namespace LED
{

class ILed;
using LedPtr = std::shared_ptr<ILed>;
using LedVector = std::vector<LedPtr>;

class ILedBehavior;
using LedBehaviorPtr = std::shared_ptr<ILedBehavior>;

/*!
 * \brief Types of behaviors for LED arrays
 */
enum class LedCommandType
{
  /*!
   * \brief All LEDs on at full brightness
   */
  FULL_BRIGHTNESS,

  /*!
   * \brief Brightness pattern that uses an LED array to indicate progress is
   *        being made
   */
  BUSY_SIGNAL,

  /*!
   * \brief Brightness pattern that uses an LED array to mimic a progress bar
   */
  PROGRESS_BAR,
};

}
}
