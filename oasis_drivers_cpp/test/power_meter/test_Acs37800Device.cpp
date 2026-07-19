/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "power_meter/Acs37800Device.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

using OASIS::PowerMeter::Acs37800Config;
using OASIS::PowerMeter::Acs37800Device;
using OASIS::PowerMeter::DecodeActivePower;
using OASIS::PowerMeter::DecodeOvercurrent;
using OASIS::PowerMeter::DecodeRmsCurrent;
using OASIS::PowerMeter::DecodeRmsVoltage;
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
    auto script = scripted_reads.find(address);
    if (script != scripted_reads.end() && !script->second.empty())
    {
      const std::uint32_t value = script->second.front();
      script->second.pop_front();
      return value;
    }
    const auto entry = registers.find(address);
    if (entry == registers.end())
      throw std::runtime_error("scripted transport error");
    return entry->second;
  }

  void WriteRegister(std::uint8_t address, std::uint32_t value) override
  {
    writes.emplace_back(address, value);
    if (!ignore_writes)
      registers[address] = value;
  }

  std::map<std::uint8_t, std::uint32_t> registers;
  std::map<std::uint8_t, std::deque<std::uint32_t>> scripted_reads;
  std::vector<std::uint8_t> reads;
  std::vector<std::pair<std::uint8_t, std::uint32_t>> writes;
  bool ignore_writes{false};
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

void NoSleep(double)
{
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

TEST(Acs37800Device, DecodesRmsAndSignedActivePowerFields)
{
  const Acs37800Config config = Config();
  const double multiplier = 2008200.0 / 8200.0;

  EXPECT_DOUBLE_EQ(DecodeRmsVoltage(0, config), 0.0);
  EXPECT_NEAR(DecodeRmsVoltage(0xFFFFU, config), 0.25 * 1.19 * multiplier * 65535.0 / 65536.0,
              1.0e-12);
  EXPECT_GT(DecodeRmsVoltage(0x8000U, config), 0.0);
  EXPECT_DOUBLE_EQ(DecodeRmsCurrent(0, config), 0.0);
  EXPECT_NEAR(DecodeRmsCurrent(0x40000000U, config), 0.5 * 30.0 * 1.19, 1.0e-12);
  EXPECT_NEAR(DecodeRmsCurrent(0x7FFF0000U, config), 30.0 * 1.19 * 32767.0 / 32768.0, 1.0e-12);
  EXPECT_LT(DecodeRmsCurrent(0xFFFF0000U, config), 0.0);
  EXPECT_LT(DecodeRmsCurrent(0x80000000U, config), 0.0);
  EXPECT_EQ(DecodeRmsCurrent(0x40001234U, config), DecodeRmsCurrent(0x40000000U, config));
  EXPECT_DOUBLE_EQ(DecodeActivePower(0, config), 0.0);
  EXPECT_GT(DecodeActivePower(0x00007FFFU, config), 0.0);
  EXPECT_LT(DecodeActivePower(0x00008000U, config), 0.0);
  EXPECT_EQ(DecodeRmsVoltage(0xABCD0001U, config), DecodeRmsVoltage(1U, config));
  EXPECT_EQ(DecodeActivePower(0xABCD0001U, config), DecodeActivePower(1U, config));
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
  config = Config();
  config.rms_sample_count = 3;
  EXPECT_THROW(ValidateAcs37800Config(config), std::invalid_argument);
  config.rms_sample_count = 1024;
  EXPECT_THROW(ValidateAcs37800Config(config), std::invalid_argument);
  config.rms_sample_count = 4;
  EXPECT_NO_THROW(ValidateAcs37800Config(config));
  config.rms_sample_count = 1023;
  EXPECT_NO_THROW(ValidateAcs37800Config(config));
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
    EXPECT_GT(DecodeRmsCurrent(0x40000000U, config), 0.0);
  }
}

TEST(Acs37800Device, ReadsSequentialRegistersAndReturnsPhysicalSample)
{
  auto transport = std::make_unique<FakeRegisterDevice>();
  FakeRegisterDevice* observer = transport.get();
  transport->registers[OASIS::PowerMeter::ACS37800_SHADOW_CURRENT_CONFIG_REGISTER] =
      CurrentConfig(4);
  transport->registers[OASIS::PowerMeter::ACS37800_SHADOW_RMS_CONFIG_REGISTER] = 0;
  transport->registers[OASIS::PowerMeter::ACS37800_RMS_VOLTAGE_CURRENT_REGISTER] = 0x40004000U;
  transport->registers[OASIS::PowerMeter::ACS37800_ACTIVE_POWER_REGISTER] = 0x0000F000U;
  transport->registers[OASIS::PowerMeter::ACS37800_FAULT_STATUS_REGISTER] = 1U << 1U;

  Acs37800Device device(std::move(transport), Config(), NoSleep);
  EXPECT_EQ(device.GetInfo().crs_sns, 4U);
  const auto sample = device.ReadSample();

  EXPECT_GT(sample.voltage, 0.0);
  EXPECT_GT(sample.current, 0.0);
  EXPECT_LT(sample.power, 0.0);
  EXPECT_TRUE(sample.overcurrent);
  EXPECT_EQ(sample.status, Status::Ok);
  EXPECT_DOUBLE_EQ(sample.voltage_variance, 0.0);
  EXPECT_EQ(observer->reads, (std::vector<std::uint8_t>{0x1B, 0x1F, 0x1F, 0x20, 0x21, 0x20, 0x2D}));
}

TEST(Acs37800Device, RejectsUnexpectedHardwareCrsSns)
{
  auto transport = std::make_unique<FakeRegisterDevice>();
  transport->registers[OASIS::PowerMeter::ACS37800_SHADOW_CURRENT_CONFIG_REGISTER] =
      CurrentConfig(2);
  try
  {
    Acs37800Device device(std::move(transport), Config(), NoSleep);
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
  transport->registers[OASIS::PowerMeter::ACS37800_SHADOW_RMS_CONFIG_REGISTER] = 0;
  Acs37800Device device(std::move(transport), Config(), NoSleep);
  EXPECT_THROW(device.ReadSample(), std::runtime_error);
}

TEST(Acs37800Device, RetriesMismatchedRmsPairThenAcceptsCoherentSample)
{
  auto transport = std::make_unique<FakeRegisterDevice>();
  FakeRegisterDevice* observer = transport.get();
  transport->registers[OASIS::PowerMeter::ACS37800_SHADOW_CURRENT_CONFIG_REGISTER] =
      CurrentConfig(4);
  transport->registers[OASIS::PowerMeter::ACS37800_SHADOW_RMS_CONFIG_REGISTER] = 0;
  transport->scripted_reads[OASIS::PowerMeter::ACS37800_RMS_VOLTAGE_CURRENT_REGISTER] = {1U, 2U, 3U,
                                                                                         3U};
  transport->registers[OASIS::PowerMeter::ACS37800_ACTIVE_POWER_REGISTER] = 0;
  transport->registers[OASIS::PowerMeter::ACS37800_FAULT_STATUS_REGISTER] = 0;

  Acs37800Device device(std::move(transport), Config(), NoSleep);
  const auto sample = device.ReadSample();
  EXPECT_EQ(sample.raw_voltage_current_register, 3U);
  EXPECT_EQ(std::count(observer->reads.begin(), observer->reads.end(), 0x20), 4);
  EXPECT_EQ(device.GetDiagnostics().coherent_read_attempts, 2U);
  EXPECT_EQ(device.GetDiagnostics().successful_first_attempt_reads, 0U);
  EXPECT_EQ(device.GetDiagnostics().coherence_retries, 1U);
  EXPECT_EQ(device.GetDiagnostics().coherence_retry_exhaustions, 0U);
}

TEST(Acs37800Device, ExhaustsBoundedCoherenceRetriesWithoutReturningStaleData)
{
  auto transport = std::make_unique<FakeRegisterDevice>();
  FakeRegisterDevice* observer = transport.get();
  transport->registers[OASIS::PowerMeter::ACS37800_SHADOW_CURRENT_CONFIG_REGISTER] =
      CurrentConfig(4);
  transport->registers[OASIS::PowerMeter::ACS37800_SHADOW_RMS_CONFIG_REGISTER] = 0;
  transport->scripted_reads[OASIS::PowerMeter::ACS37800_RMS_VOLTAGE_CURRENT_REGISTER] = {
      1U, 2U, 3U, 4U, 5U, 6U};
  transport->registers[OASIS::PowerMeter::ACS37800_ACTIVE_POWER_REGISTER] = 0;

  Acs37800Device device(std::move(transport), Config(), NoSleep);
  EXPECT_THROW(device.ReadSample(), std::runtime_error);
  EXPECT_EQ(std::count(observer->reads.begin(), observer->reads.end(), 0x20), 6);
  EXPECT_EQ(std::count(observer->reads.begin(), observer->reads.end(), 0x21), 3);
  EXPECT_EQ(device.GetDiagnostics().coherent_read_attempts, 3U);
  EXPECT_EQ(device.GetDiagnostics().coherence_retries, 3U);
  EXPECT_EQ(device.GetDiagnostics().coherence_retry_exhaustions, 1U);
}

TEST(Acs37800Device, RejectsFixedRmsConfigurationReadbackMismatch)
{
  auto transport = std::make_unique<FakeRegisterDevice>();
  transport->registers[OASIS::PowerMeter::ACS37800_SHADOW_CURRENT_CONFIG_REGISTER] =
      CurrentConfig(4);
  transport->registers[OASIS::PowerMeter::ACS37800_SHADOW_RMS_CONFIG_REGISTER] = 0;
  transport->ignore_writes = true;
  EXPECT_THROW(Acs37800Device(std::move(transport), Config(), NoSleep), std::runtime_error);
}

TEST(Acs37800Device, PreservesUnrelatedRmsConfigFieldsAndWaitsForTwoWindows)
{
  auto transport = std::make_unique<FakeRegisterDevice>();
  FakeRegisterDevice* observer = transport.get();
  transport->registers[OASIS::PowerMeter::ACS37800_SHADOW_CURRENT_CONFIG_REGISTER] =
      CurrentConfig(4);
  constexpr std::uint32_t unrelated_fields = 0x02003555U;
  transport->registers[OASIS::PowerMeter::ACS37800_SHADOW_RMS_CONFIG_REGISTER] = unrelated_fields;
  double slept_seconds = 0.0;

  Acs37800Device device(std::move(transport), Config(),
                        [&slept_seconds](double seconds) { slept_seconds = seconds; });

  ASSERT_EQ(observer->writes.size(), 1U);
  const std::uint32_t changed_mask = (0x3FFU << 14U) | (1U << 24U);
  EXPECT_EQ(observer->writes.front().second & ~changed_mask, unrelated_fields & ~changed_mask);
  EXPECT_EQ(observer->writes.front().second & changed_mask, (0x3FFU << 14U) | (1U << 24U));
  EXPECT_DOUBLE_EQ(slept_seconds, 2.0 * 1023.0 / 32000.0);
}

TEST(Acs37800Device, RejectsNegativeRmsCurrentSample)
{
  auto transport = std::make_unique<FakeRegisterDevice>();
  transport->registers[OASIS::PowerMeter::ACS37800_SHADOW_CURRENT_CONFIG_REGISTER] =
      CurrentConfig(4);
  transport->registers[OASIS::PowerMeter::ACS37800_SHADOW_RMS_CONFIG_REGISTER] = 0;
  transport->registers[OASIS::PowerMeter::ACS37800_RMS_VOLTAGE_CURRENT_REGISTER] = 0xFFFF0000U;
  transport->registers[OASIS::PowerMeter::ACS37800_ACTIVE_POWER_REGISTER] = 0;
  transport->registers[OASIS::PowerMeter::ACS37800_FAULT_STATUS_REGISTER] = 0;
  Acs37800Device device(std::move(transport), Config(), NoSleep);
  EXPECT_THROW(device.ReadSample(), std::runtime_error);
}
