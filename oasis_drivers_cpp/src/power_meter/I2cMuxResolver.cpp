/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "I2cMuxResolver.hpp"

#include <algorithm>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <system_error>

namespace
{
std::string HexAddress(int address)
{
  std::ostringstream stream;
  stream << "0x" << std::hex << address;
  return stream.str();
}

std::vector<std::filesystem::directory_entry> DirectoryEntries(const std::filesystem::path& path,
                                                               const std::string& purpose)
{
  std::error_code error;
  std::filesystem::directory_iterator iterator(path, error);
  if (error)
    throw std::runtime_error("Failed to inspect " + path.string() + " for " + purpose + ": " +
                             error.message());

  std::vector<std::filesystem::directory_entry> entries;
  const std::filesystem::directory_iterator end;
  while (iterator != end)
  {
    entries.push_back(*iterator);
    iterator.increment(error);
    if (error)
      throw std::runtime_error("Failed while inspecting " + path.string() + ": " + error.message());
  }
  return entries;
}

int ParseAdapterNumber(const std::filesystem::path& target, const std::filesystem::path& link_path)
{
  static const std::regex adapter_pattern(R"(^i2c-([0-9]+)$)");
  std::smatch match;
  const std::string target_name = target.filename().string();
  if (!std::regex_match(target_name, match, adapter_pattern))
  {
    throw std::runtime_error("Mux channel link " + link_path.string() +
                             " does not target an i2c-N adapter");
  }
  return std::stoi(match[1].str());
}
} // namespace

OASIS::PowerMeter::MuxDevice OASIS::PowerMeter::DiscoverMuxDevice(
    const std::filesystem::path& sysfs_root, int mux_address)
{
  static const std::regex device_pattern(R"(^([0-9]+)-([0-9a-fA-F]{4})$)");
  std::vector<MuxDevice> matches;
  for (const std::filesystem::directory_entry& entry :
       DirectoryEntries(sysfs_root, "PCA9548 address " + HexAddress(mux_address)))
  {
    std::smatch match;
    const std::string name = entry.path().filename().string();
    if (!std::regex_match(name, match, device_pattern))
      continue;
    const int address = std::stoi(match[2].str(), nullptr, 16);
    if (address == mux_address)
      matches.push_back({std::stoi(match[1].str()), address, entry.path()});
  }

  if (matches.empty())
    throw std::runtime_error("No PCA9548 mux device found under " + sysfs_root.string() +
                             " at address " + HexAddress(mux_address));
  if (matches.size() != 1)
    throw std::runtime_error("Multiple PCA9548 mux devices found under " + sysfs_root.string() +
                             " at the configured address");
  return matches.front();
}

std::vector<OASIS::PowerMeter::MuxChannelAdapter> OASIS::PowerMeter::DiscoverMuxChannels(
    const MuxDevice& mux)
{
  static const std::regex channel_pattern(R"(^channel-([0-9]+)$)");
  std::vector<MuxChannelAdapter> channels;
  for (const std::filesystem::directory_entry& entry :
       DirectoryEntries(mux.sysfs_path, "PCA9548 child adapter links"))
  {
    std::smatch match;
    const std::string entry_name = entry.path().filename().string();
    if (!std::regex_match(entry_name, match, channel_pattern))
      continue;

    std::error_code error;
    const std::filesystem::path target = std::filesystem::canonical(entry.path(), error);
    if (error)
      throw std::runtime_error("Failed to follow mux channel link " + entry.path().string() + ": " +
                               error.message());
    const int channel = std::stoi(match[1].str());
    if (channel < 0 || channel > 7)
      throw std::runtime_error("Invalid PCA9548 channel in link " + entry.path().string());
    const int adapter_number = ParseAdapterNumber(target, entry.path());
    channels.push_back(
        {mux.parent_bus, channel, adapter_number, "/dev/i2c-" + std::to_string(adapter_number)});
  }

  std::sort(channels.begin(), channels.end(),
            [](const auto& left, const auto& right) { return left.channel < right.channel; });
  for (std::size_t index = 1; index < channels.size(); ++index)
  {
    if (channels[index - 1].channel == channels[index].channel ||
        channels[index - 1].adapter_number == channels[index].adapter_number)
    {
      throw std::runtime_error("Duplicate or ambiguous PCA9548 channel links under " +
                               mux.sysfs_path.string());
    }
  }
  if (channels.empty())
    throw std::runtime_error("No PCA9548 child adapter links found under " +
                             mux.sysfs_path.string());
  return channels;
}

std::vector<OASIS::PowerMeter::MuxChannelAdapter> OASIS::PowerMeter::ProbeDevicesOnMux(
    const std::vector<MuxChannelAdapter>& channels, int device_address, II2cDeviceProbe& probe)
{
  std::vector<MuxChannelAdapter> devices;
  for (const MuxChannelAdapter& channel : channels)
  {
    const I2cProbeResult result = probe.Probe(channel.device_path, device_address);
    if (result.status == I2cProbeStatus::Present)
    {
      devices.push_back(channel);
      continue;
    }
    if (result.status == I2cProbeStatus::Error)
    {
      throw std::runtime_error("I2C probe failed for mux channel " +
                               std::to_string(channel.channel) + " via " + channel.device_path +
                               " at address " + HexAddress(device_address) + ": " + result.error);
    }
  }
  return devices;
}
