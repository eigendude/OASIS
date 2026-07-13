/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "power_meter/I2cMuxResolver.hpp"
#include "power_meter/RaspberryPiI2cConfig.hpp"

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

using OASIS::PowerMeter::DiscoverMuxChannels;
using OASIS::PowerMeter::DiscoverMuxDevice;
using OASIS::PowerMeter::I2cProbeResult;
using OASIS::PowerMeter::I2cProbeStatus;
using OASIS::PowerMeter::II2cDeviceProbe;
using OASIS::PowerMeter::ParseRaspberryPiI2cConfig;
using OASIS::PowerMeter::ProbeDevicesOnMux;

namespace
{
class FakeI2cDeviceProbe : public II2cDeviceProbe
{
public:
  I2cProbeResult Probe(const std::string& device_path, int device_address) override
  {
    calls.emplace_back(device_path, device_address);
    const auto result = results.find(device_path);
    return result != results.end() ? result->second
                                   : I2cProbeResult{I2cProbeStatus::NotPresent, {}};
  }

  std::map<std::string, I2cProbeResult> results;
  std::vector<std::pair<std::string, int>> calls;
};

class FakeI2cFilesystem : public testing::Test
{
protected:
  void SetUp() override
  {
    const auto unique_id = std::chrono::steady_clock::now().time_since_epoch().count();
    m_root = std::filesystem::temp_directory_path() /
             ("oasis_i2c_discovery_" + std::to_string(unique_id));
    m_sysfs = m_root / "sys";
    m_config = m_root / "config.txt";
    std::filesystem::create_directories(m_sysfs);
  }

  void TearDown() override { std::filesystem::remove_all(m_root); }

  void WriteConfig(const std::string& contents) { std::ofstream(m_config) << contents; }

  void AddMux(int parent_bus, int address)
  {
    char address_text[5];
    std::snprintf(address_text, sizeof(address_text), "%04x", address);
    m_mux = m_sysfs / (std::to_string(parent_bus) + "-" + address_text);
    std::filesystem::create_directories(m_mux);
  }

  void AddChannel(int channel, int adapter)
  {
    const std::filesystem::path adapter_path = m_sysfs / ("i2c-" + std::to_string(adapter));
    std::filesystem::create_directories(adapter_path);
    std::filesystem::create_directory_symlink(std::filesystem::path("..") / adapter_path.filename(),
                                              m_mux / ("channel-" + std::to_string(channel)));
  }

  std::filesystem::path m_root;
  std::filesystem::path m_sysfs;
  std::filesystem::path m_config;
  std::filesystem::path m_mux;
};

TEST_F(FakeI2cFilesystem, ParsesWhitespaceCommentsSectionsAndNumericAddresses)
{
  WriteConfig("# disabled\n"
              "  # dtoverlay=i2c-mux,pca9548,addr=0x71\n"
              "[pi4]\n"
              " dtoverlay = i2c-mux, pca9548, addr=112 # PCA9548\n"
              "[all]\n");
  EXPECT_EQ(ParseRaspberryPiI2cConfig(m_config, 0x70).mux_address, 0x70);

  WriteConfig("dtoverlay=i2c-mux,pca9548,addr=0x70\n");
  EXPECT_EQ(ParseRaspberryPiI2cConfig(m_config, 0x70).mux_address, 0x70);
}

TEST_F(FakeI2cFilesystem, RejectsMissingMalformedMismatchedAndConflictingOverlays)
{
  WriteConfig("dtoverlay=other-device\n");
  EXPECT_THROW(ParseRaspberryPiI2cConfig(m_config, 0x70), std::runtime_error);

  WriteConfig("dtoverlay=i2c-mux,pca9548\n");
  EXPECT_THROW(ParseRaspberryPiI2cConfig(m_config, 0x70), std::runtime_error);

  WriteConfig("dtoverlay=i2c-mux,pca9548,addr=0x71\n");
  EXPECT_THROW(ParseRaspberryPiI2cConfig(m_config, 0x70), std::runtime_error);

  WriteConfig("dtoverlay=i2c-mux,pca9548,addr=0x70\n"
              "dtoverlay=i2c-mux,pca9548,addr=0x71\n");
  EXPECT_THROW(ParseRaspberryPiI2cConfig(m_config, 0x70), std::runtime_error);
  EXPECT_THROW(ParseRaspberryPiI2cConfig(m_root / "missing.txt", 0x70), std::runtime_error);
}

TEST_F(FakeI2cFilesystem, FollowsMuxLinksAndSortsChannels)
{
  AddMux(8, 0x70);
  AddChannel(3, 107);
  AddChannel(1, 9);
  AddChannel(2, 41);

  const auto mux = DiscoverMuxDevice(m_sysfs, 0x70);
  EXPECT_EQ(mux.parent_bus, 8);
  const auto channels = DiscoverMuxChannels(mux);
  ASSERT_EQ(channels.size(), 3U);
  EXPECT_EQ(channels[0].channel, 1);
  EXPECT_EQ(channels[1].channel, 2);
  EXPECT_EQ(channels[2].channel, 3);
}

TEST_F(FakeI2cFilesystem, ProbesInChannelOrderAndSkipsNotPresentDevices)
{
  AddMux(8, 0x70);
  AddChannel(3, 4);
  AddChannel(1, 9);
  AddChannel(2, 99);
  const auto mux = DiscoverMuxDevice(m_sysfs, 0x70);
  const auto channels = DiscoverMuxChannels(mux);

  FakeI2cDeviceProbe probe;
  probe.results["/dev/i2c-99"] = {I2cProbeStatus::Present, {}};
  probe.results["/dev/i2c-4"] = {I2cProbeStatus::Present, {}};
  const auto meters = ProbeDevicesOnMux(channels, 0x60, probe);

  ASSERT_EQ(meters.size(), 2U);
  EXPECT_EQ(meters[0].channel, 2);
  EXPECT_EQ(meters[0].adapter_number, 99);
  EXPECT_EQ(meters[0].device_path, "/dev/i2c-99");
  EXPECT_EQ(meters[1].channel, 3);
  EXPECT_EQ(meters[1].adapter_number, 4);

  ASSERT_EQ(probe.calls.size(), 3U);
  EXPECT_EQ(probe.calls[0], std::make_pair(std::string("/dev/i2c-9"), 0x60));
  EXPECT_EQ(probe.calls[1], std::make_pair(std::string("/dev/i2c-99"), 0x60));
  EXPECT_EQ(probe.calls[2], std::make_pair(std::string("/dev/i2c-4"), 0x60));
}

TEST_F(FakeI2cFilesystem, ProbeErrorIncludesChannelPathAndAddress)
{
  AddMux(1, 0x70);
  AddChannel(2, 22);
  const auto channels = DiscoverMuxChannels(DiscoverMuxDevice(m_sysfs, 0x70));
  FakeI2cDeviceProbe probe;
  probe.results["/dev/i2c-22"] = {I2cProbeStatus::Error, "permission denied"};

  try
  {
    (void)ProbeDevicesOnMux(channels, 0x60, probe);
    FAIL() << "Expected a fatal probe error";
  }
  catch (const std::runtime_error& error)
  {
    const std::string message = error.what();
    EXPECT_NE(message.find("channel 2"), std::string::npos);
    EXPECT_NE(message.find("/dev/i2c-22"), std::string::npos);
    EXPECT_NE(message.find("0x60"), std::string::npos);
  }
}

TEST_F(FakeI2cFilesystem, RejectsAbsentMux)
{
  EXPECT_THROW(DiscoverMuxDevice(m_sysfs, 0x70), std::runtime_error);
}
} // namespace
