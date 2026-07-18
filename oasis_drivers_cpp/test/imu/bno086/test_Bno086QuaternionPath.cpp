/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/core/Bno086RotationVectorDecoder.hpp"
#include "imu/bno086/ros/Bno086MessageBuilder.hpp"
#include "imu/bno086/sh2/Bno086Shtp.hpp"
#include "imu/bno086/utils/Bno086MathUtils.hpp"

#include <array>
#include <cstdint>
#include <deque>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <rclcpp/time.hpp>

using namespace OASIS::IMU::BNO086;

namespace
{
constexpr std::array<std::int16_t, 4> kRawQ14{4096, -6144, 8192, 12288};
constexpr double kTolerance = 1.0e-12;

class PacketTransport : public Bno086Transport
{
public:
  bool WritePacket(std::uint8_t, const std::vector<std::uint8_t>&) override { return true; }

  bool ReadPacket(Bno086ShtpPacket& packet, int) override
  {
    if (packets.empty())
      return false;
    packet = packets.front();
    packets.pop_front();
    return true;
  }

  std::deque<Bno086ShtpPacket> packets;
};

void AppendS16(std::vector<std::uint8_t>& payload, std::int16_t value)
{
  const std::uint16_t raw = static_cast<std::uint16_t>(value);
  payload.push_back(static_cast<std::uint8_t>(raw & 0xFFU));
  payload.push_back(static_cast<std::uint8_t>((raw >> 8) & 0xFFU));
}

SensorEvent DecodeRawRotationVector()
{
  std::vector<std::uint8_t> payload{static_cast<std::uint8_t>(ReportId::RotationVector), 7, 3, 0};
  for (const std::int16_t component : kRawQ14)
    AppendS16(payload, component);
  AppendS16(payload, 256);

  PacketTransport transport;
  transport.packets.push_back(Bno086ShtpPacket{3, 0, false, payload});
  Bno086Shtp shtp(transport);
  const Bno086Shtp::PollResult result = shtp.Poll(5);
  EXPECT_EQ(result.status, Bno086Shtp::PollStatus::SensorEvent);
  EXPECT_TRUE(result.event.has_value());
  return result.event.value_or(SensorEvent{});
}

Eigen::Quaterniond ExpectedWorldFromImu()
{
  Eigen::Quaterniond expected(0.75, 0.25, -0.375, 0.5);
  return expected.normalized();
}

Eigen::Quaterniond FromXyzw(const std::array<double, 4>& value)
{
  return Eigen::Quaterniond(value[3], value[0], value[1], value[2]);
}

void ExpectSameRotation(const Eigen::Quaterniond& actual, const Eigen::Quaterniond& expected)
{
  const Eigen::Vector3d vectors[] = {Eigen::Vector3d(0.31, -0.72, 0.18),
                                     Eigen::Vector3d(-0.44, 0.12, 0.83)};
  for (const Eigen::Vector3d& vector : vectors)
    EXPECT_TRUE((actual * vector).isApprox(expected * vector, kTolerance));
}
} // namespace

TEST(Bno086QuaternionPath, RawSh2IjkRealQ14DecodesAsImuToWorldRotation)
{
  const SensorEvent event = DecodeRawRotationVector();

  EXPECT_EQ(event.values[0], 4096);
  EXPECT_EQ(event.values[1], -6144);
  EXPECT_EQ(event.values[2], 8192);
  EXPECT_EQ(event.values[3], 12288);
  EXPECT_DOUBLE_EQ(QToDouble(event.values[0], 14), 0.25);
  EXPECT_DOUBLE_EQ(QToDouble(event.values[1], 14), -0.375);
  EXPECT_DOUBLE_EQ(QToDouble(event.values[2], 14), 0.5);
  EXPECT_DOUBLE_EQ(QToDouble(event.values[3], 14), 0.75);

  const auto decoded = DecodeRotationVectorWorldFromImu(event);
  ASSERT_TRUE(decoded.has_value());
  const Eigen::Quaterniond actual = FromXyzw(*decoded);
  const Eigen::Quaterniond expected = ExpectedWorldFromImu();
  ExpectSameRotation(actual, expected);
  EXPECT_NEAR(actual.norm(), 1.0, kTolerance);
  EXPECT_GT(
      (actual * Eigen::Vector3d::UnitX() - expected.conjugate() * Eigen::Vector3d::UnitX()).norm(),
      0.5);
}

TEST(Bno086QuaternionPath, RawRotationVectorPublishesUnchangedImuToWorldRotation)
{
  const SensorEvent event = DecodeRawRotationVector();
  const auto decoded = DecodeRotationVectorWorldFromImu(event);
  ASSERT_TRUE(decoded.has_value());

  ImuSampleFrame frame;
  frame.has_orientation = true;
  frame.orientation_world_from_imu_xyzw = *decoded;
  OASIS::ROS::Bno086RosMessageConfig config;
  config.frame_id = "imu_link";
  const sensor_msgs::msg::Imu message =
      OASIS::ROS::BuildBno086PresentImuMessage(config, frame, rclcpp::Time{});
  const Eigen::Quaterniond published(message.orientation.w, message.orientation.x,
                                     message.orientation.y, message.orientation.z);

  EXPECT_EQ(message.header.frame_id, "imu_link");
  ExpectSameRotation(published, ExpectedWorldFromImu());

  const Eigen::Quaterniond swapped(message.orientation.w, message.orientation.y,
                                   message.orientation.x, -message.orientation.z);
  EXPECT_GT(
      (published * Eigen::Vector3d(0.2, 0.7, -0.4) - swapped * Eigen::Vector3d(0.2, 0.7, -0.4))
          .norm(),
      0.2);
}

TEST(Bno086QuaternionPath, DecoderRejectsNonRotationVectorEventWithoutQuaternion)
{
  SensorEvent unrelated_event;
  unrelated_event.report_id = ReportId::Gravity;
  unrelated_event.values = {4096, -6144, 8192, 12288, 0};

  const auto decoded = DecodeRotationVectorWorldFromImu(unrelated_event);

  EXPECT_FALSE(decoded.has_value());
}
