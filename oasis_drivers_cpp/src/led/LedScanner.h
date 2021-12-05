/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "LedTypes.h"

namespace rclcpp
{
class Logger;
}

namespace OASIS
{
namespace LED
{

/*!
 * \brief Class to scan for LEDs
 */
class LedScanner
{
public:
  /*!
   * \brief Do a scan and return the discovered LEDs
   *
   * \return The LEDs in order of the physical array, or empty if no LEDs were
   *         discovered.
   */
  static LedVector GetLEDs(rclcpp::Logger& logger);
};

} // namespace LED
} // namespace OASIS
