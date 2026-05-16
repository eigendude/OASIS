/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/Bno086Shtp.hpp"

#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

using namespace OASIS::IMU::BNO086;

namespace
{
class RecordingTransport : public Bno086Transport
{
public:
  bool WritePacket(std::uint8_t channel, const std::vector<std::uint8_t>& payload) override
  {
    channels.emplace_back(channel);
    payloads.emplace_back(payload);
    return true;
  }

  std::vector<std::uint8_t> channels;
  std::vector<std::vector<std::uint8_t>> payloads;
};

std::uint32_t ReadIntervalUs(const std::vector<std::uint8_t>& payload)
{
  return static_cast<std::uint32_t>(payload[5]) | (static_cast<std::uint32_t>(payload[6]) << 8) |
         (static_cast<std::uint32_t>(payload[7]) << 16) |
         (static_cast<std::uint32_t>(payload[8]) << 24);
}
} // namespace

TEST(Bno086Shtp, configuresStaticPerReportIntervals)
{
  RecordingTransport transport;
  Bno086Shtp shtp{transport};

  Bno086ShtpConfig config;
  config.report_rate_hz = 10.0;
  config.rotation_vector_rate_hz = 100.0;
  config.gyro_rate_hz = 100.0;
  config.accelerometer_rate_hz = 100.0;
  config.linear_acceleration_rate_hz = 50.0;
  config.gravity_rate_hz = 25.0;

  ASSERT_TRUE(shtp.Configure(config));

  const std::vector<FeatureConfiguration>& features = shtp.GetFeatureConfigurations();
  ASSERT_EQ(features.size(), 5U);
  EXPECT_EQ(features[0].report_id, ReportId::RotationVector);
  EXPECT_EQ(features[0].requested_interval_us, 10'000U);
  EXPECT_EQ(features[1].report_id, ReportId::GyroscopeCalibrated);
  EXPECT_EQ(features[1].requested_interval_us, 10'000U);
  EXPECT_EQ(features[2].report_id, ReportId::LinearAcceleration);
  EXPECT_EQ(features[2].requested_interval_us, 20'000U);
  EXPECT_EQ(features[3].report_id, ReportId::Accelerometer);
  EXPECT_EQ(features[3].requested_interval_us, 10'000U);
  EXPECT_EQ(features[4].report_id, ReportId::Gravity);
  EXPECT_EQ(features[4].requested_interval_us, 40'000U);

  ASSERT_GE(transport.payloads.size(), 9U);
  EXPECT_EQ(ReadIntervalUs(transport.payloads[0]), 10'000U);
  EXPECT_EQ(ReadIntervalUs(transport.payloads[2]), 10'000U);
  EXPECT_EQ(ReadIntervalUs(transport.payloads[4]), 20'000U);
  EXPECT_EQ(ReadIntervalUs(transport.payloads[6]), 10'000U);
  EXPECT_EQ(ReadIntervalUs(transport.payloads[8]), 40'000U);
}

TEST(Bno086Shtp, omitsDisabledGravityReport)
{
  RecordingTransport transport;
  Bno086Shtp shtp{transport};

  Bno086ShtpConfig config;
  config.enable_gravity_report = false;

  ASSERT_TRUE(shtp.Configure(config));

  const std::vector<FeatureConfiguration>& features = shtp.GetFeatureConfigurations();
  ASSERT_EQ(features.size(), 4U);
  for (const FeatureConfiguration& feature : features)
    EXPECT_NE(feature.report_id, ReportId::Gravity);
}
