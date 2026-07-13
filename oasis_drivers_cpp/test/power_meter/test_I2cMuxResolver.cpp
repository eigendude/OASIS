/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "power_meter/I2cMuxResolver.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

using OASIS::PowerMeter::AdapterResolution;
using OASIS::PowerMeter::ResolveMuxAdapter;
using OASIS::PowerMeter::ResolveMuxAdapterIfEnabled;

namespace
{
class FakeI2cSysfs : public testing::Test
{
protected:
  void SetUp() override
  {
    const auto unique_id = std::chrono::steady_clock::now().time_since_epoch().count();
    m_root = std::filesystem::temp_directory_path() /
             ("oasis_i2c_mux_resolver_" + std::to_string(unique_id));
    std::filesystem::create_directories(m_root);
  }

  void TearDown() override { std::filesystem::remove_all(m_root); }

  void AddAdapter(int adapter_number, const std::string& name)
  {
    const std::filesystem::path adapter_path = m_root / ("i2c-" + std::to_string(adapter_number));
    std::filesystem::create_directories(adapter_path);
    std::ofstream(adapter_path / "name") << name << '\n';
  }

  std::filesystem::path m_root;
};

TEST_F(FakeI2cSysfs, ResolvesIndependentChannelsToDynamicAdapterNumbers)
{
  AddAdapter(4, "DesignWare I2C adapter");
  AddAdapter(41, "i2c-1-mux (chan_id 2)");
  AddAdapter(107, "pca954x i2c-1-mux (chan_id 3)");

  const auto channel_two = ResolveMuxAdapterIfEnabled(m_root, true, 1, 0x70, 2);
  const AdapterResolution channel_three = ResolveMuxAdapter(m_root, 1, 0x70, 3);

  ASSERT_TRUE(channel_two.has_value());
  EXPECT_EQ(channel_two->adapter_number, 41);
  EXPECT_EQ(channel_two->device_path, "/dev/i2c-41");
  EXPECT_EQ(channel_three.adapter_number, 107);
  EXPECT_EQ(channel_three.device_path, "/dev/i2c-107");
}

TEST_F(FakeI2cSysfs, IgnoresUnrelatedAndMalformedEntries)
{
  AddAdapter(12, "i2c-8-mux (chan_id 2)");
  AddAdapter(13, "not a mux adapter name");
  std::filesystem::create_directories(m_root / "i2c-not-a-number");
  std::filesystem::create_directories(m_root / "i2c-14");
  AddAdapter(87, "i2c-1-mux (chan_id 2)");

  const AdapterResolution resolution = ResolveMuxAdapter(m_root, 1, 0x70, 2);

  EXPECT_EQ(resolution.adapter_number, 87);
}

TEST_F(FakeI2cSysfs, EnforcesParentBusMatching)
{
  AddAdapter(41, "i2c-8-mux (chan_id 2)");

  EXPECT_THROW(ResolveMuxAdapter(m_root, 1, 0x70, 2), std::runtime_error);
}

TEST_F(FakeI2cSysfs, DisabledResolutionDoesNotRequireSysfsMuxEntry)
{
  const auto resolution = ResolveMuxAdapterIfEnabled(m_root / "missing", false, 1, 0x70, 2);

  EXPECT_FALSE(resolution.has_value());
}

TEST_F(FakeI2cSysfs, MissingChannelReportsCompleteMuxIdentity)
{
  AddAdapter(41, "i2c-1-mux (chan_id 4)");

  try
  {
    (void)ResolveMuxAdapter(m_root, 1, 0x70, 2);
    FAIL() << "Expected missing mux channel to throw";
  }
  catch (const std::runtime_error& error)
  {
    const std::string message = error.what();
    EXPECT_NE(message.find("parent bus 1"), std::string::npos);
    EXPECT_NE(message.find("mux address 0x70"), std::string::npos);
    EXPECT_NE(message.find("mux channel 2"), std::string::npos);
  }
}
} // namespace
