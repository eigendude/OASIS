/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "environment/Bme280Compensation.h"
#include "environment/Ens160Device.h"
#include "environment/RunningVariance.h"

#include <array>
#include <cstdint>

#include <gtest/gtest.h>

using OASIS::Environment::Bme280Calibration;
using OASIS::Environment::Bme280Compensation;
using OASIS::Environment::Bme280RawSample;
using OASIS::Environment::Ens160Device;
using OASIS::Environment::Ens160Validity;
using OASIS::Environment::RunningVariance;

namespace
{
void WriteU16Le(std::array<std::uint8_t, 26>& data, std::size_t offset, std::uint16_t value)
{
  data[offset] = static_cast<std::uint8_t>(value & 0xFFU);
  data[offset + 1] = static_cast<std::uint8_t>(value >> 8);
}

Bme280Calibration BoschReferenceCalibration()
{
  Bme280Calibration calibration;
  calibration.dig_t1 = 27504;
  calibration.dig_t2 = 26435;
  calibration.dig_t3 = -1000;
  calibration.dig_p1 = 36477;
  calibration.dig_p2 = -10685;
  calibration.dig_p3 = 3024;
  calibration.dig_p4 = 2855;
  calibration.dig_p5 = 140;
  calibration.dig_p6 = -7;
  calibration.dig_p7 = 15500;
  calibration.dig_p8 = -14600;
  calibration.dig_p9 = 6000;
  calibration.dig_h1 = 75;
  calibration.dig_h2 = 362;
  calibration.dig_h3 = 0;
  calibration.dig_h4 = 325;
  calibration.dig_h5 = 50;
  calibration.dig_h6 = 30;
  return calibration;
}
} // namespace

TEST(Bme280Compensation, DecodesSignedUnsignedAndPackedCalibration)
{
  std::array<std::uint8_t, 26> primary{};
  std::array<std::uint8_t, 7> humidity{};
  WriteU16Le(primary, 0, 27504);
  WriteU16Le(primary, 2, static_cast<std::uint16_t>(-26435));
  WriteU16Le(primary, 6, 36477);
  primary[25] = 75;
  humidity[0] = 0x6A;
  humidity[1] = 0x01;
  humidity[2] = 7;
  humidity[3] = 0xEC;
  humidity[4] = 0xF3;
  humidity[5] = 0xFF;
  humidity[6] = 0xE2;

  const Bme280Calibration decoded = Bme280Compensation::DecodeCalibration(primary, humidity);
  EXPECT_EQ(decoded.dig_t1, 27504);
  EXPECT_EQ(decoded.dig_t2, -26435);
  EXPECT_EQ(decoded.dig_p1, 36477);
  EXPECT_EQ(decoded.dig_h1, 75);
  EXPECT_EQ(decoded.dig_h2, 362);
  EXPECT_EQ(decoded.dig_h3, 7);
  EXPECT_EQ(decoded.dig_h4, -317);
  EXPECT_EQ(decoded.dig_h5, -1);
  EXPECT_EQ(decoded.dig_h6, -30);
}

TEST(Bme280Compensation, DecodesRawAdcFields)
{
  const std::array<std::uint8_t, 8> data{0x65, 0x5A, 0xC0, 0x7E, 0xED, 0x00, 0x7D, 0x01};
  const Bme280RawSample raw = Bme280Compensation::DecodeRawSample(data);
  EXPECT_EQ(raw.pressure, 415148U);
  EXPECT_EQ(raw.temperature, 519888U);
  EXPECT_EQ(raw.humidity, 32001U);
}

TEST(Bme280Compensation, MatchesBoschReferenceTemperatureAndPressure)
{
  const auto sample = Bme280Compensation::Compensate(BoschReferenceCalibration(),
                                                     Bme280RawSample{415148, 519888, 32257});
  EXPECT_NEAR(sample.temperature_c, 25.082, 0.001);
  EXPECT_NEAR(sample.pressure_pa, 100653.27, 0.02);
  EXPECT_NEAR(sample.relative_humidity_percent, 63.272, 0.01);
  EXPECT_NEAR(sample.relative_humidity_percent / 100.0, 0.63272, 0.0001);
}

TEST(Ens160Device, DecodesPartIdStatusAndLittleEndianOutput)
{
  EXPECT_EQ(Ens160Device::DecodePartId({0x60, 0x01}), 0x0160);
  const auto status = Ens160Device::DecodeStatus(0x0A);
  EXPECT_EQ(status.validity, Ens160Validity::InitialStartup);
  EXPECT_TRUE(status.new_data);
  EXPECT_EQ(status.error, 0);

  const auto sample = Ens160Device::DecodeSample({5, 0xE8, 0x03, 0x90, 0x01});
  EXPECT_EQ(sample.air_quality_index, 5);
  EXPECT_EQ(sample.tvoc_ppb, 1000);
  EXPECT_EQ(sample.equivalent_co2_ppm, 400);
  EXPECT_DOUBLE_EQ(Ens160Device::TvocPpbToPpm(sample.tvoc_ppb), 1.0);
  EXPECT_TRUE(Ens160Device::IsSampleValid(status, sample));
}

TEST(Ens160Device, ValidatesRangesAndRejectsInvalidStatus)
{
  const auto normal = Ens160Device::DecodeStatus(0x02);
  EXPECT_TRUE(Ens160Device::IsSampleValid(normal, {1, 400, 0}));
  EXPECT_TRUE(Ens160Device::IsSampleValid(normal, {5, 65000, 65000}));
  EXPECT_FALSE(Ens160Device::IsSampleValid(normal, {0, 400, 0}));
  EXPECT_FALSE(Ens160Device::IsSampleValid(normal, {6, 400, 0}));
  EXPECT_FALSE(Ens160Device::IsSampleValid(normal, {1, 399, 0}));
  EXPECT_FALSE(Ens160Device::IsSampleValid(Ens160Device::DecodeStatus(0x0E), {1, 400, 0}));
}

TEST(Ens160Device, EncodesEnvironmentalCompensation)
{
  EXPECT_EQ(Ens160Device::EncodeTemperature(25.0), 19082);
  EXPECT_EQ(Ens160Device::EncodeHumidity(50.0), 25600);
}

TEST(RunningVariance, UsesSampleVarianceAndEvictsOldestValue)
{
  RunningVariance variance(3);
  EXPECT_DOUBLE_EQ(variance.Variance(), 0.0);
  variance.Add(1.0);
  EXPECT_DOUBLE_EQ(variance.Variance(), 0.0);
  variance.Add(2.0);
  variance.Add(3.0);
  EXPECT_DOUBLE_EQ(variance.Variance(), 1.0);
  variance.Add(4.0);
  EXPECT_DOUBLE_EQ(variance.Variance(), 1.0);
  variance.Reset();
  EXPECT_EQ(variance.Size(), 0U);
  EXPECT_DOUBLE_EQ(variance.Variance(), 0.0);
}

TEST(RunningVariance, RemainsStableForLargeOffsets)
{
  RunningVariance variance(3);
  variance.Add(1.0e12 + 1.0);
  variance.Add(1.0e12 + 2.0);
  variance.Add(1.0e12 + 3.0);
  EXPECT_NEAR(variance.Variance(), 1.0, 1.0e-9);
}
