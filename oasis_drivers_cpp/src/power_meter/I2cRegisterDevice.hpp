/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <cstdint>

namespace OASIS::PowerMeter
{
/** \brief Transport-independent interface for 32-bit I2C registers */
class II2cRegisterDevice
{
public:
  virtual ~II2cRegisterDevice() = default;

  /** \brief Read one register and return its host-order 32-bit value */
  virtual std::uint32_t ReadRegister(std::uint8_t address) = 0;

  /** \brief Write one host-order 32-bit register value */
  virtual void WriteRegister(std::uint8_t address, std::uint32_t value) = 0;
};
} // namespace OASIS::PowerMeter
