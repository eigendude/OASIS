/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <filesystem>

namespace OASIS::PowerMeter
{
/** \brief Effective Raspberry Pi PCA9548 boot-overlay configuration */
struct RaspberryPiI2cConfig
{
  //! Configured PCA9548 7-bit I2C address, in the range 0x03 to 0x77
  int mux_address{-1};
};

/** \brief Parse and validate the enabled PCA9548 overlay in config.txt */
RaspberryPiI2cConfig ParseRaspberryPiI2cConfig(const std::filesystem::path& config_path,
                                               int expected_mux_address);
} // namespace OASIS::PowerMeter
