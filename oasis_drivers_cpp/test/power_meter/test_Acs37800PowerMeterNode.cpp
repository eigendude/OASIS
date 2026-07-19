/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "nodes/Acs37800PowerMeterNode.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <oasis_msgs/msg/power_meter.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

using OASIS::PowerMeter::I2cProbeResult;
using OASIS::PowerMeter::I2cProbeStatus;
using OASIS::PowerMeter::IAcs37800Device;
using OASIS::PowerMeter::II2cDeviceProbe;
using OASIS::PowerMeter::Sample;
using OASIS::PowerMeter::Status;
using OASIS::ROS::Acs37800PowerMeterNode;

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

struct FakeDeviceState
{
  double voltage{0.0};
  Status status{Status::Ok};
  bool fail{false};
  unsigned reads{0};
  std::deque<Sample> scripted_samples;
};

class FakeDevice : public IAcs37800Device
{
public:
  explicit FakeDevice(std::shared_ptr<FakeDeviceState> state) : m_state(std::move(state))
  {
    m_info.crs_sns = 4;
    m_info.voltage_multiplier = 2008200.0 / 8200.0;
  }

  Sample ReadSample() override
  {
    ++m_state->reads;
    if (m_state->fail)
      throw std::runtime_error("scripted read failure");
    if (!m_state->scripted_samples.empty())
    {
      const Sample sample = m_state->scripted_samples.front();
      m_state->scripted_samples.pop_front();
      return sample;
    }
    Sample sample;
    sample.voltage = m_state->voltage;
    sample.current = 0.5;
    sample.power = m_state->voltage * 0.5;
    sample.status = m_state->status;
    return sample;
  }

  const OASIS::PowerMeter::Acs37800DeviceInfo& GetInfo() const override { return m_info; }
  const OASIS::PowerMeter::Acs37800Diagnostics& GetDiagnostics() const override
  {
    return m_diagnostics;
  }

private:
  std::shared_ptr<FakeDeviceState> m_state;
  OASIS::PowerMeter::Acs37800DeviceInfo m_info{};
  OASIS::PowerMeter::Acs37800Diagnostics m_diagnostics{};
};

class PowerMeterNodeTest : public testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    const auto unique_id = std::chrono::steady_clock::now().time_since_epoch().count();
    m_root = std::filesystem::temp_directory_path() /
             ("oasis_power_meter_node_" + std::to_string(unique_id));
    m_sysfs = m_root / "sys";
    m_mux = m_sysfs / "8-0070";
    m_config = m_root / "config.txt";
    std::filesystem::create_directories(m_mux);
    std::ofstream(m_config) << "[all]\ndtoverlay=i2c-mux,pca9548,addr=0x70\n";
  }

  void TearDown() override { std::filesystem::remove_all(m_root); }

  void AddChannel(int channel, int adapter)
  {
    const auto adapter_path = m_sysfs / ("i2c-" + std::to_string(adapter));
    std::filesystem::create_directories(adapter_path);
    std::filesystem::create_directory_symlink(std::filesystem::path("..") / adapter_path.filename(),
                                              m_mux / ("channel-" + std::to_string(channel)));
  }

  rclcpp::NodeOptions Options(const std::vector<std::string>& power_meter_ids = {"power_meter_0",
                                                                                 "power_meter_1"},
                              std::int64_t filter_length = 3)
  {
    return rclcpp::NodeOptions().parameter_overrides({
        rclcpp::Parameter("boot_config_path", m_config.string()),
        rclcpp::Parameter("i2c_sysfs_root", m_sysfs.string()),
        rclcpp::Parameter("publish_rate_hz", 100.0),
        rclcpp::Parameter("mux_address", static_cast<std::int64_t>(0x70)),
        rclcpp::Parameter("power_meter_address", static_cast<std::int64_t>(0x60)),
        rclcpp::Parameter("power_meter_ids", power_meter_ids),
        rclcpp::Parameter("current_sense_range_amps", 30.0),
        rclcpp::Parameter("voltage_divider_resistance_ohms", 2000000.0),
        rclcpp::Parameter("voltage_sense_resistance_ohms", 8200.0),
        rclcpp::Parameter("expected_crs_sns", static_cast<std::int64_t>(4)),
        rclcpp::Parameter("disconnect_after_failures", static_cast<std::int64_t>(2)),
        rclcpp::Parameter("filter_length", filter_length),
    });
  }

  Acs37800PowerMeterNode::DeviceFactory Factory(
      const std::map<std::string, std::shared_ptr<FakeDeviceState>>& states)
  {
    return [states](const std::string& path, int, const OASIS::PowerMeter::Acs37800Config&)
    {
      const auto state = states.find(path);
      if (state == states.end())
        throw std::runtime_error("missing fake device");
      return std::make_unique<FakeDevice>(state->second);
    };
  }

  std::filesystem::path m_root;
  std::filesystem::path m_sysfs;
  std::filesystem::path m_mux;
  std::filesystem::path m_config;
};
} // namespace

TEST_F(PowerMeterNodeTest, DiscoversByChannelAndPublishesInSharedTimerOrder)
{
  AddChannel(3, 4);
  AddChannel(1, 9);
  AddChannel(2, 99);
  auto probe = std::make_unique<FakeI2cDeviceProbe>();
  FakeI2cDeviceProbe* probe_observer = probe.get();
  probe->results["/dev/i2c-99"] = {I2cProbeStatus::Present, {}};
  probe->results["/dev/i2c-4"] = {I2cProbeStatus::Present, {}};
  auto first_state = std::make_shared<FakeDeviceState>();
  first_state->voltage = 12.0;
  auto second_state = std::make_shared<FakeDeviceState>();
  second_state->voltage = 11.8;
  second_state->status = Status::Stale;
  const auto node = std::make_shared<Acs37800PowerMeterNode>(
      Options({"channel_2_meter", "channel_3_meter"}), std::move(probe),
      Factory({{"/dev/i2c-99", first_state}, {"/dev/i2c-4", second_state}}));

  EXPECT_EQ(node->get_parameter("power_meter_ids").as_string_array(),
            (std::vector<std::string>{"channel_2_meter", "channel_3_meter"}));
  EXPECT_FALSE(node->has_parameter("parent_i2c_bus"));
  EXPECT_EQ(node->get_parameter("mux_address").as_int(), 0x70);
  EXPECT_EQ(node->get_parameter("power_meter_address").as_int(), 0x60);
  EXPECT_DOUBLE_EQ(node->get_parameter("current_sense_range_amps").as_double(), 30.0);
  EXPECT_EQ(node->get_parameter("expected_crs_sns").as_int(), 4);
  EXPECT_FALSE(node->has_parameter("expected_current_coarse_gain"));
  ASSERT_EQ(probe_observer->calls.size(), 3U);
  EXPECT_EQ(probe_observer->calls[0], std::make_pair(std::string("/dev/i2c-9"), 0x60));
  EXPECT_EQ(probe_observer->calls[1], std::make_pair(std::string("/dev/i2c-99"), 0x60));
  EXPECT_EQ(probe_observer->calls[2], std::make_pair(std::string("/dev/i2c-4"), 0x60));

  const auto observer = std::make_shared<rclcpp::Node>("power_meter_observer");
  std::vector<double> first_voltages;
  std::vector<double> second_voltages;
  std::vector<std::string> first_frame_ids;
  std::vector<std::string> second_frame_ids;
  std::vector<rclcpp::Time> first_timestamps;
  std::vector<rclcpp::Time> second_timestamps;
  std::vector<oasis_msgs::msg::PowerMeter> first_messages;
  std::vector<oasis_msgs::msg::PowerMeter> second_messages;
  const auto first_subscription = observer->create_subscription<oasis_msgs::msg::PowerMeter>(
      "/channel_2_meter", rclcpp::SensorDataQoS(),
      [&](const oasis_msgs::msg::PowerMeter& message)
      {
        first_messages.push_back(message);
        first_voltages.push_back(message.voltage);
        first_frame_ids.push_back(message.header.frame_id);
        first_timestamps.emplace_back(message.header.stamp);
      });
  const auto second_subscription = observer->create_subscription<oasis_msgs::msg::PowerMeter>(
      "/channel_3_meter", rclcpp::SensorDataQoS(),
      [&](const oasis_msgs::msg::PowerMeter& message)
      {
        second_messages.push_back(message);
        second_voltages.push_back(message.voltage);
        second_frame_ids.push_back(message.header.frame_id);
        second_timestamps.emplace_back(message.header.stamp);
      });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(observer);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while ((first_voltages.empty() || second_voltages.empty()) &&
         std::chrono::steady_clock::now() < deadline)
    executor.spin_some();

  ASSERT_FALSE(first_voltages.empty());
  ASSERT_FALSE(second_voltages.empty());
  EXPECT_EQ(first_timestamps.front(), second_timestamps.front());
  EXPECT_DOUBLE_EQ(first_voltages.front(), 12.0);
  EXPECT_DOUBLE_EQ(second_voltages.front(), 11.8);
  EXPECT_EQ(first_frame_ids.front(), "channel_2_meter");
  EXPECT_EQ(second_frame_ids.front(), "channel_3_meter");
  EXPECT_DOUBLE_EQ(first_messages.front().voltage_variance, 0.0);
  EXPECT_DOUBLE_EQ(first_messages.front().current_variance, 0.0);
  EXPECT_DOUBLE_EQ(first_messages.front().power_variance, 0.0);
  EXPECT_DOUBLE_EQ(second_messages.front().voltage_variance, 0.0);
  EXPECT_DOUBLE_EQ(second_messages.front().current_variance, 0.0);
  EXPECT_DOUBLE_EQ(second_messages.front().power_variance, 0.0);
  EXPECT_EQ(first_messages.front().status, oasis_msgs::msg::PowerMeter::STATUS_OK);
  EXPECT_EQ(second_messages.front().status, oasis_msgs::msg::PowerMeter::STATUS_STALE);
  (void)first_subscription;
  (void)second_subscription;
}

TEST_F(PowerMeterNodeTest, IsolatesReadFailureAndPublishesExplicitError)
{
  AddChannel(2, 22);
  AddChannel(3, 23);
  auto probe = std::make_unique<FakeI2cDeviceProbe>();
  probe->results["/dev/i2c-22"] = {I2cProbeStatus::Present, {}};
  probe->results["/dev/i2c-23"] = {I2cProbeStatus::Present, {}};
  auto failed_state = std::make_shared<FakeDeviceState>();
  failed_state->fail = true;
  auto good_state = std::make_shared<FakeDeviceState>();
  good_state->voltage = 24.0;
  const auto node = std::make_shared<Acs37800PowerMeterNode>(
      Options(), std::move(probe),
      Factory({{"/dev/i2c-22", failed_state}, {"/dev/i2c-23", good_state}}));

  const auto observer = std::make_shared<rclcpp::Node>("failure_observer");
  std::vector<oasis_msgs::msg::PowerMeter> failed_messages;
  std::vector<oasis_msgs::msg::PowerMeter> good_messages;
  const auto failed_subscription = observer->create_subscription<oasis_msgs::msg::PowerMeter>(
      "/power_meter_0", rclcpp::SensorDataQoS(),
      [&](const oasis_msgs::msg::PowerMeter& message) { failed_messages.push_back(message); });
  const auto good_subscription = observer->create_subscription<oasis_msgs::msg::PowerMeter>(
      "/power_meter_1", rclcpp::SensorDataQoS(),
      [&](const oasis_msgs::msg::PowerMeter& message) { good_messages.push_back(message); });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(observer);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while ((failed_messages.size() < 3 || good_messages.empty()) &&
         std::chrono::steady_clock::now() < deadline)
    executor.spin_some();

  ASSERT_GE(failed_messages.size(), 3U);
  ASSERT_FALSE(good_messages.empty());
  for (std::size_t index = 0; index < 3; ++index)
  {
    EXPECT_TRUE(std::isnan(failed_messages[index].voltage));
    EXPECT_TRUE(std::isnan(failed_messages[index].current));
    EXPECT_TRUE(std::isnan(failed_messages[index].power));
    EXPECT_DOUBLE_EQ(failed_messages[index].voltage_variance, 0.0);
    EXPECT_DOUBLE_EQ(failed_messages[index].current_variance, 0.0);
    EXPECT_DOUBLE_EQ(failed_messages[index].power_variance, 0.0);
  }
  EXPECT_EQ(failed_messages.front().status, oasis_msgs::msg::PowerMeter::STATUS_ERROR);
  EXPECT_EQ(failed_messages[1].status, oasis_msgs::msg::PowerMeter::STATUS_DISCONNECTED);
  EXPECT_EQ(failed_messages[2].status, oasis_msgs::msg::PowerMeter::STATUS_DISCONNECTED);
  EXPECT_DOUBLE_EQ(good_messages.front().voltage, 24.0);
  EXPECT_DOUBLE_EQ(good_messages.front().voltage_variance, 0.0);
  EXPECT_DOUBLE_EQ(good_messages.front().current_variance, 0.0);
  EXPECT_DOUBLE_EQ(good_messages.front().power_variance, 0.0);
  EXPECT_EQ(good_messages.front().status, oasis_msgs::msg::PowerMeter::STATUS_OK);
  EXPECT_EQ(failed_messages.front().header.stamp, good_messages.front().header.stamp);

  const std::size_t recovery_start = failed_messages.size();
  failed_state->fail = false;
  const auto recovery_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (std::none_of(failed_messages.begin() + recovery_start, failed_messages.end(),
                      [](const oasis_msgs::msg::PowerMeter& message)
                      { return message.status == oasis_msgs::msg::PowerMeter::STATUS_OK; }) &&
         std::chrono::steady_clock::now() < recovery_deadline)
  {
    executor.spin_some();
  }
  const auto recovered =
      std::find_if(failed_messages.begin() + recovery_start, failed_messages.end(),
                   [](const oasis_msgs::msg::PowerMeter& message)
                   { return message.status == oasis_msgs::msg::PowerMeter::STATUS_OK; });
  ASSERT_NE(recovered, failed_messages.end());
  EXPECT_DOUBLE_EQ(recovered->voltage_variance, 0.0);
  EXPECT_DOUBLE_EQ(recovered->current_variance, 0.0);
  EXPECT_DOUBLE_EQ(recovered->power_variance, 0.0);

  const std::size_t new_failure_start = failed_messages.size();
  failed_state->fail = true;
  const auto new_failure_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (failed_messages.size() == new_failure_start &&
         std::chrono::steady_clock::now() < new_failure_deadline)
  {
    executor.spin_some();
  }
  ASSERT_GT(failed_messages.size(), new_failure_start);
  EXPECT_EQ(failed_messages[new_failure_start].status, oasis_msgs::msg::PowerMeter::STATUS_ERROR);
  (void)failed_subscription;
  (void)good_subscription;
}

TEST_F(PowerMeterNodeTest, RejectsNoMatchingMeters)
{
  AddChannel(2, 22);
  auto probe = std::make_unique<FakeI2cDeviceProbe>();
  EXPECT_THROW(std::make_shared<Acs37800PowerMeterNode>(Options(), std::move(probe), Factory({})),
               std::runtime_error);
}

TEST_F(PowerMeterNodeTest, RejectsPowerMeterIdCountMismatch)
{
  AddChannel(2, 22);
  AddChannel(3, 23);
  auto probe = std::make_unique<FakeI2cDeviceProbe>();
  probe->results["/dev/i2c-22"] = {I2cProbeStatus::Present, {}};
  probe->results["/dev/i2c-23"] = {I2cProbeStatus::Present, {}};

  try
  {
    (void)std::make_shared<Acs37800PowerMeterNode>(Options({"only_one_id"}), std::move(probe),
                                                   Factory({}));
    FAIL() << "Expected power_meter_ids count validation to fail";
  }
  catch (const std::invalid_argument& error)
  {
    EXPECT_STREQ(error.what(),
                 "power_meter_ids contains 1 IDs, but 2 power meters were discovered");
  }
}

TEST_F(PowerMeterNodeTest, RejectsTooManyPowerMeterIds)
{
  AddChannel(2, 22);
  AddChannel(3, 23);
  auto probe = std::make_unique<FakeI2cDeviceProbe>();
  probe->results["/dev/i2c-22"] = {I2cProbeStatus::Present, {}};
  probe->results["/dev/i2c-23"] = {I2cProbeStatus::Present, {}};

  try
  {
    (void)std::make_shared<Acs37800PowerMeterNode>(Options({"first", "second", "extra"}),
                                                   std::move(probe), Factory({}));
    FAIL() << "Expected power_meter_ids count validation to fail";
  }
  catch (const std::invalid_argument& error)
  {
    EXPECT_STREQ(error.what(),
                 "power_meter_ids contains 3 IDs, but 2 power meters were discovered");
  }
}

TEST_F(PowerMeterNodeTest, RejectsEmptyPowerMeterId)
{
  AddChannel(2, 22);
  auto probe = std::make_unique<FakeI2cDeviceProbe>();
  probe->results["/dev/i2c-22"] = {I2cProbeStatus::Present, {}};

  EXPECT_THROW(
      (void)std::make_shared<Acs37800PowerMeterNode>(Options({""}), std::move(probe), Factory({})),
      std::invalid_argument);
}

TEST_F(PowerMeterNodeTest, RejectsDuplicatePowerMeterId)
{
  AddChannel(2, 22);
  AddChannel(3, 23);
  auto probe = std::make_unique<FakeI2cDeviceProbe>();
  probe->results["/dev/i2c-22"] = {I2cProbeStatus::Present, {}};
  probe->results["/dev/i2c-23"] = {I2cProbeStatus::Present, {}};

  EXPECT_THROW((void)std::make_shared<Acs37800PowerMeterNode>(Options({"duplicate", "duplicate"}),
                                                              std::move(probe), Factory({})),
               std::invalid_argument);
}

TEST_F(PowerMeterNodeTest, FatalProbeErrorAbortsStartup)
{
  AddChannel(2, 22);
  auto probe = std::make_unique<FakeI2cDeviceProbe>();
  probe->results["/dev/i2c-22"] = {I2cProbeStatus::Error, "adapter access denied"};
  EXPECT_THROW(std::make_shared<Acs37800PowerMeterNode>(Options(), std::move(probe), Factory({})),
               std::runtime_error);
}

TEST_F(PowerMeterNodeTest, PublishesFilteredValues)
{
  AddChannel(2, 22);
  auto probe = std::make_unique<FakeI2cDeviceProbe>();
  probe->results["/dev/i2c-22"] = {I2cProbeStatus::Present, {}};
  auto state = std::make_shared<FakeDeviceState>();
  for (double voltage : {1.0, 2.0, 3.0})
  {
    Sample sample;
    sample.voltage = voltage;
    sample.current = 1.0;
    sample.power = voltage;
    sample.status = Status::Ok;
    state->scripted_samples.push_back(sample);
  }
  const auto node = std::make_shared<Acs37800PowerMeterNode>(
      Options({"filtered_meter"}, 3), std::move(probe), Factory({{"/dev/i2c-22", state}}));
  const auto observer = std::make_shared<rclcpp::Node>("filter_observer");
  std::vector<double> voltages;
  const auto subscription = observer->create_subscription<oasis_msgs::msg::PowerMeter>(
      "/filtered_meter", rclcpp::SensorDataQoS(),
      [&](const oasis_msgs::msg::PowerMeter& message) { voltages.push_back(message.voltage); });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(observer);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (voltages.size() < 3 && std::chrono::steady_clock::now() < deadline)
    executor.spin_some();
  ASSERT_GE(voltages.size(), 3U);
  EXPECT_DOUBLE_EQ(voltages[0], 1.0);
  EXPECT_DOUBLE_EQ(voltages[1], 1.5);
  EXPECT_DOUBLE_EQ(voltages[2], 2.0);
  (void)subscription;
}
