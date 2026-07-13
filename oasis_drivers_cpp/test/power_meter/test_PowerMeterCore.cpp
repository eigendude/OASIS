/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "power_meter/PowerMeterCore.hpp"

#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

using OASIS::PowerMeter::BuildSimulatedSample;
using OASIS::PowerMeter::Config;
using OASIS::PowerMeter::Sample;
using OASIS::PowerMeter::Status;
using OASIS::PowerMeter::ValidateConfig;

namespace
{
void ExpectValidationError(const Config& config, const std::string& expected_message)
{
  try
  {
    ValidateConfig(config);
    FAIL() << "Expected invalid power-meter configuration";
  }
  catch (const std::invalid_argument& error)
  {
    EXPECT_EQ(error.what(), expected_message);
  }
}
} // namespace

TEST(PowerMeterCore, BuildsAggregateSimulatedSample)
{
  Config config;
  config.simulated_voltage = 12.0;
  config.simulated_current = 0.5;
  config.voltage_variance = 0.01;
  config.current_variance = 0.01;
  config.simulated_overcurrent = true;

  const Sample sample = BuildSimulatedSample(config);

  EXPECT_DOUBLE_EQ(sample.voltage, 12.0);
  EXPECT_DOUBLE_EQ(sample.current, 0.5);
  EXPECT_DOUBLE_EQ(sample.power, 6.0);
  EXPECT_DOUBLE_EQ(sample.power_variance, 1.4425);
  EXPECT_TRUE(sample.overcurrent);
  EXPECT_EQ(sample.status, Status::Simulated);
  EXPECT_EQ(static_cast<std::uint8_t>(sample.status), 1);
}

TEST(PowerMeterCore, AcceptsDocumentedParameterDefaults)
{
  EXPECT_TRUE(Config{}.resolve_i2c_adapter);
  EXPECT_NO_THROW(ValidateConfig(Config{}));
}

TEST(PowerMeterCore, RejectsNanSimulatedVoltage)
{
  Config config;
  config.simulated_voltage = std::numeric_limits<double>::quiet_NaN();

  ExpectValidationError(config, "simulated_voltage must be finite");
}

TEST(PowerMeterCore, RejectsInfiniteSimulatedVoltage)
{
  Config config;
  config.simulated_voltage = std::numeric_limits<double>::infinity();

  ExpectValidationError(config, "simulated_voltage must be finite");
}

TEST(PowerMeterCore, RejectsNanSimulatedCurrent)
{
  Config config;
  config.simulated_current = std::numeric_limits<double>::quiet_NaN();

  ExpectValidationError(config, "simulated_current must be finite");
}

TEST(PowerMeterCore, RejectsInfiniteSimulatedCurrent)
{
  Config config;
  config.simulated_current = std::numeric_limits<double>::infinity();

  ExpectValidationError(config, "simulated_current must be finite");
}

TEST(PowerMeterCore, RejectsInvalidBusMuxAddressChannelAndDeviceAddress)
{
  Config config;
  config.parent_i2c_bus = -1;
  EXPECT_THROW(ValidateConfig(config), std::invalid_argument);

  config = Config{};
  config.mux_address = 0x02;
  EXPECT_THROW(ValidateConfig(config), std::invalid_argument);

  config = Config{};
  config.mux_address = 0x78;
  EXPECT_THROW(ValidateConfig(config), std::invalid_argument);

  config = Config{};
  config.mux_channel = 8;
  EXPECT_THROW(ValidateConfig(config), std::invalid_argument);

  config = Config{};
  config.i2c_address = 0x02;
  EXPECT_THROW(ValidateConfig(config), std::invalid_argument);

  config = Config{};
  config.i2c_address = 0x78;
  EXPECT_THROW(ValidateConfig(config), std::invalid_argument);
}

TEST(PowerMeterCore, RejectsInvalidRateAndVariances)
{
  Config config;
  config.publish_rate_hz = 0.0;
  EXPECT_THROW(ValidateConfig(config), std::invalid_argument);

  config = Config{};
  config.publish_rate_hz = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(ValidateConfig(config), std::invalid_argument);

  config = Config{};
  config.voltage_variance = -0.01;
  EXPECT_THROW(ValidateConfig(config), std::invalid_argument);

  config = Config{};
  config.current_variance = -0.01;
  EXPECT_THROW(ValidateConfig(config), std::invalid_argument);
}
