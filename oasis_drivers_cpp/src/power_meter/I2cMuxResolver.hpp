/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "I2cDeviceProbe.hpp"

#include <filesystem>
#include <string>
#include <vector>

namespace OASIS::PowerMeter
{
/** \brief PCA9548 device discovered under the Linux I2C sysfs hierarchy */
struct MuxDevice
{
  //! Linux parent I2C bus number parsed from the mux device entry
  int parent_bus{-1};

  //! PCA9548 7-bit I2C address parsed from the mux device entry
  int address{-1};

  //! Mux device entry below the supplied sysfs root
  std::filesystem::path sysfs_path;
};

/** \brief Linux child adapter associated with one PCA9548 mux channel */
struct MuxChannelAdapter
{
  //! Linux bus containing the parent PCA9548 device
  int parent_bus{-1};

  //! PCA9548 channel number, in the inclusive range 0 to 7
  int channel{-1};

  //! Dynamic Linux I2C adapter number assigned to the channel
  int adapter_number{-1};

  //! Character-device path derived from adapter_number
  std::string device_path;
};

/** \brief Find the unique PCA9548 device at the configured address */
MuxDevice DiscoverMuxDevice(const std::filesystem::path& sysfs_root, int mux_address);

/** \brief Follow mux channel links and return child adapters by channel */
std::vector<MuxChannelAdapter> DiscoverMuxChannels(const MuxDevice& mux);

/** \brief Probe sorted mux channels and return those containing the device */
std::vector<MuxChannelAdapter> ProbeDevicesOnMux(const std::vector<MuxChannelAdapter>& channels,
                                                 int device_address,
                                                 II2cDeviceProbe& probe);
} // namespace OASIS::PowerMeter
