/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "power_meter/Acs37800Device.hpp"

#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

using OASIS::PowerMeter::Acs37800Config;
using OASIS::PowerMeter::Acs37800Device;
using OASIS::PowerMeter::DecodeCurrent;
using OASIS::PowerMeter::DecodeOvercurrent;
using OASIS::PowerMeter::DecodePower;
using OASIS::PowerMeter::DecodeVoltage;
using OASIS::PowerMeter::II2cRegisterDevice;
using OASIS::PowerMeter::SignExtend;
using OASIS::PowerMeter::Status;
using OASIS::PowerMeter::ValidateAcs37800Config;

namespace
{
class FakeRegisterDevice : public II2cRegisterDevice
{
public:
  std::uint32_t ReadRegister(std::uint8_t address) override
  {
    reads.push_back(address);
    const auto entry = registers.find(address);
    if (entry == registers.end())
      throw std::runtime_error("scripted transport error");
    return entry->second;
  }

  std::map<std::uint8_t, std::uint32_t> registers;
  std::vector<std::uint8_t> reads;
};

Acs37800Config Config()
{
  return {
      .current_sense_range_amps = 30.0,
      .voltage_divider_resistance_ohms = 2000000.0,
      .voltage_sense_resistance_ohms = 8200.0,
      .expected_crs_sns = 4,
  };
}

std::uint32_t CurrentConfig(unsigned crs_sns)
{
  return crs_sns << 19U;
}
} // namespace

TEST(Acs37800Device, SignExtendsBoundaries)
{
  EXPECT_EQ(SignExtend(0x0000, 16), 0);
  EXPECT_EQ(SignExtend(0x7FFF, 16), 32767);
  EXPECT_EQ(SignExtend(0x8000, 16), -32768);
  EXPECT_EQ(SignExtend(0xFFFF, 16), -1);
  EXPECT_THROW(SignExtend(0, 0), std::invalid_argument);
}

TEST(Acs37800Device, DecodesSignedInstantaneousFieldsAndScaling)
{
  const Acs37800Config config = Config();
  const double multiplier = 2008200.0 / 8200.0;

  EXPECT_DOUBLE_EQ(DecodeVoltage(0, config), 0.0);
  EXPECT_NEAR(DecodeVoltage(27500, config), 0.250 * multiplier, 1.0e-12);
  EXPECT_NEAR(DecodeVoltage(static_cast<std::uint16_t>(-27500), config), -0.250 * multiplier,
              1.0e-12);
  EXPECT_DOUBLE_EQ(DecodeCurrent(0, config), 0.0);
  EXPECT_NEAR(DecodeCurrent(27500, config), 30.0, 1.0e-12);
  EXPECT_NEAR(DecodeCurrent(static_cast<std::uint16_t>(-27500), config), -30.0, 1.0e-12);
  EXPECT_DOUBLE_EQ(DecodePower(0, config), 0.0);
  EXPECT_LT(DecodePower(static_cast<std::uint16_t>(-1000), config), 0.0);
  EXPECT_GT(DecodePower(1000, config), 0.0);
  EXPECT_GT(DecodeVoltage(0x7FFF, config), 0.0);
  EXPECT_LT(DecodeVoltage(0x8000, config), 0.0);
  EXPECT_GT(DecodeCurrent(0x7FFF, config), 0.0);
  EXPECT_LT(DecodeCurrent(0x8000, config), 0.0);
  EXPECT_GT(DecodePower(0x7FFF, config), 0.0);
  EXPECT_LT(DecodePower(0x8000, config), 0.0);
}

TEST(Acs37800Device, ExtractsOnlyLiveFaultBit)
{
  EXPECT_FALSE(DecodeOvercurrent(0));
  EXPECT_TRUE(DecodeOvercurrent(1U << 1U));
  EXPECT_FALSE(DecodeOvercurrent(1U << 2U));
}

TEST(Acs37800Device, RejectsInvalidPhysicalConfiguration)
{
  Acs37800Config config = Config();
  config.current_sense_range_amps = 20.0;
  EXPECT_THROW(ValidateAcs37800Config(config), std::invalid_argument);
  config = Config();
  config.voltage_divider_resistance_ohms = 0.0;
  EXPECT_THROW(ValidateAcs37800Config(config), std::invalid_argument);
  config = Config();
  config.voltage_sense_resistance_ohms = NAN;
  EXPECT_THROW(ValidateAcs37800Config(config), std::invalid_argument);
  config = Config();
  config.expected_crs_sns = 8;
  EXPECT_THROW(ValidateAcs37800Config(config), std::invalid_argument);
}

TEST(Acs37800Device, AcceptsExpectedCrsSnsAcrossRegisterFieldRange)
{
  Acs37800Config config = Config();
  for (unsigned crs_sns = 0; crs_sns <= 7; ++crs_sns)
  {
    config.expected_crs_sns = crs_sns;
    EXPECT_NO_THROW(ValidateAcs37800Config(config));
  }
}

TEST(Acs37800Device, CurrentRangeIsIndependentOfExpectedCrsSns)
{
  Acs37800Config config = Config();
  EXPECT_DOUBLE_EQ(config.current_sense_range_amps, 30.0);
  for (unsigned crs_sns = 0; crs_sns <= 7; ++crs_sns)
  {
    config.expected_crs_sns = crs_sns;
    EXPECT_DOUBLE_EQ(DecodeCurrent(27500, config), 30.0);
    EXPECT_DOUBLE_EQ(DecodeCurrent(static_cast<std::uint16_t>(-27500), config), -30.0);
  }
}

TEST(Acs37800Device, ReadsSequentialRegistersAndReturnsPhysicalSample)
{
  auto transport = std::make_unique<FakeRegisterDevice>();
  FakeRegisterDevice* observer = transport.get();
  transport->registers[OASIS::PowerMeter::ACS37800_SHADOW_CURRENT_CONFIG_REGISTER] =
      CurrentConfig(4);
  transport->registers[OASIS::PowerMeter::ACS37800_INSTANTANEOUS_VOLTAGE_CURRENT_REGISTER] =
      (static_cast<std::uint32_t>(static_cast<std::uint16_t>(-13750)) << 16U) | 13750U;
  transport->registers[OASIS::PowerMeter::ACS37800_INSTANTANEOUS_POWER_REGISTER] =
      static_cast<std::uint16_t>(-1000);
  transport->registers[OASIS::PowerMeter::ACS37800_FAULT_STATUS_REGISTER] = 1U << 1U;

  Acs37800Device device(std::move(transport), Config());
  EXPECT_EQ(device.GetInfo().crs_sns, 4U);
  const auto sample = device.ReadSample();

  EXPECT_NEAR(sample.voltage, 0.125 * 2008200.0 / 8200.0, 1.0e-12);
  EXPECT_DOUBLE_EQ(sample.current, -15.0);
  EXPECT_LT(sample.power, 0.0);
  EXPECT_TRUE(sample.overcurrent);
  EXPECT_EQ(sample.status, Status::Ok);
  EXPECT_DOUBLE_EQ(sample.voltage_variance, 0.0);
  EXPECT_EQ(observer->reads, (std::vector<std::uint8_t>{0x1B, 0x2A, 0x2C, 0x2D}));
}

TEST(Acs37800Device, RejectsUnexpectedHardwareCrsSns)
{
  auto transport = std::make_unique<FakeRegisterDevice>();
  transport->registers[OASIS::PowerMeter::ACS37800_SHADOW_CURRENT_CONFIG_REGISTER] =
      CurrentConfig(2);
  try
  {
    Acs37800Device device(std::move(transport), Config());
    FAIL() << "Expected startup crs_sns mismatch";
  }
  catch (const std::runtime_error& error)
  {
    EXPECT_STREQ(error.what(), "ACS37800 crs_sns is 2, expected 4");
  }
}

TEST(Acs37800Device, PropagatesTransportFailureWithoutStaleSample)
{
  auto transport = std::make_unique<FakeRegisterDevice>();
  transport->registers[OASIS::PowerMeter::ACS37800_SHADOW_CURRENT_CONFIG_REGISTER] =
      CurrentConfig(4);
  Acs37800Device device(std::move(transport), Config());
  EXPECT_THROW(device.ReadSample(), std::runtime_error);
}
