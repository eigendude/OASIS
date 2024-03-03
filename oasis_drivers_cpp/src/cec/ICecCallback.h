/*
 *  Copyright (C) 2021-2024 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <string>

namespace OASIS
{
namespace CEC
{

class ICecAdapter;

class ICecCallback
{
public:
  virtual ~ICecCallback() = default;

  /*!
   * \brief Called when a device is powered on
   *
   * \param adapter An interface to the device's CEC adapter
   */
  virtual void OnPowerOn(const ICecAdapter& adapter) = 0;

  /*!
   * \brief Called when a device is powered off
   *
   * \param adapter An interface to the device's CEC adapter
   */
  virtual void OnPowerOff(const ICecAdapter& adapter) = 0;
};

} // namespace CEC
} // namespace OASIS
