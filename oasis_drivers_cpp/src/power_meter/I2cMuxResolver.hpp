/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <filesystem>
#include <optional>
#include <string>

namespace OASIS::PowerMeter
{
/** \brief Linux adapter selected for a mux channel */
struct AdapterResolution
{
  //! Dynamic Linux I2C adapter number parsed from the sysfs entry name
  int adapter_number{-1};
  //! Character-device path derived from adapter_number
  std::string device_path;
};

/**
 * \brief Resolve a mux channel through Linux i2c-dev sysfs adapter names
 *
 * The mux address remains part of this API and its errors even though the
 * initial kernel adapter-name match cannot distinguish same-address muxes.
 */
AdapterResolution ResolveMuxAdapter(const std::filesystem::path& i2c_dev_sysfs_root,
                                    int parent_i2c_bus,
                                    int mux_address,
                                    int mux_channel);

/**
 * \brief Resolve a mux adapter only when startup hardware validation is enabled
 *
 * Returns no value without inspecting sysfs when resolution is disabled.
 */
std::optional<AdapterResolution> ResolveMuxAdapterIfEnabled(
    const std::filesystem::path& i2c_dev_sysfs_root,
    bool resolve_i2c_adapter,
    int parent_i2c_bus,
    int mux_address,
    int mux_channel);
} // namespace OASIS::PowerMeter
