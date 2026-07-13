/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "I2cMuxResolver.hpp"

#include <fstream>
#include <iomanip>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <system_error>

using OASIS::PowerMeter::AdapterResolution;

namespace
{
std::string DescribeMux(int parent_i2c_bus, int mux_address, int mux_channel)
{
  std::ostringstream description;
  description << "parent bus " << parent_i2c_bus << ", mux address 0x" << std::hex << std::setw(2)
              << std::setfill('0') << mux_address << std::dec << ", mux channel " << mux_channel;
  return description.str();
}
} // namespace

AdapterResolution OASIS::PowerMeter::ResolveMuxAdapter(
    const std::filesystem::path& i2c_dev_sysfs_root,
    int parent_i2c_bus,
    int mux_address,
    int mux_channel)
{
  const std::string expected_name = "i2c-" + std::to_string(parent_i2c_bus) + "-mux (chan_id " +
                                    std::to_string(mux_channel) + ")";
  const std::regex adapter_entry_pattern(R"(^i2c-([0-9]+)$)");

  std::error_code iterator_error;
  std::filesystem::directory_iterator entries(i2c_dev_sysfs_root, iterator_error);
  if (iterator_error)
  {
    throw std::runtime_error("Failed to inspect " + i2c_dev_sysfs_root.string() + " for " +
                             DescribeMux(parent_i2c_bus, mux_address, mux_channel) + ": " +
                             iterator_error.message());
  }

  for (const std::filesystem::directory_entry& entry : entries)
  {
    std::smatch adapter_match;
    const std::string entry_name = entry.path().filename().string();
    if (!std::regex_match(entry_name, adapter_match, adapter_entry_pattern))
      continue;

    std::ifstream name_file(entry.path() / "name");
    std::string adapter_name;
    if (!name_file || !std::getline(name_file, adapter_name) ||
        adapter_name.find(expected_name) == std::string::npos)
    {
      continue;
    }

    try
    {
      const int adapter_number = std::stoi(adapter_match[1].str());
      return {adapter_number, "/dev/i2c-" + std::to_string(adapter_number)};
    }
    catch (const std::exception&)
    {
      continue;
    }
  }

  throw std::runtime_error("No Linux I2C child adapter found for " +
                           DescribeMux(parent_i2c_bus, mux_address, mux_channel));
}

std::optional<AdapterResolution> OASIS::PowerMeter::ResolveMuxAdapterIfEnabled(
    const std::filesystem::path& i2c_dev_sysfs_root,
    bool resolve_i2c_adapter,
    int parent_i2c_bus,
    int mux_address,
    int mux_channel)
{
  if (!resolve_i2c_adapter)
    return std::nullopt;

  return ResolveMuxAdapter(i2c_dev_sysfs_root, parent_i2c_bus, mux_address, mux_channel);
}
