/*
 *  Copyright (C) 2025-2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "MonocularInertialSlamNode.h"

#include "slam/MonocularInertialSlam.h"

#include <algorithm>
#include <cinttypes>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <vector>

#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rmw/qos_profiles.h>

using namespace OASIS;
using namespace ROS;

namespace
{
// Subscribed topics
constexpr std::string_view IMAGE_TOPIC = "image";
constexpr std::string_view IMU_TOPIC = "imu";

// Published topics
constexpr std::string_view MAP_IMAGE_TOPIC = "slam_map_image";
constexpr std::string_view POINT_CLOUD_TOPIC = "slam_point_cloud";
constexpr std::string_view POSE_TOPIC = "slam_pose";

// Parameters
constexpr std::string_view SYSTEM_ID_PARAMETER = "system_id";
constexpr std::string_view DEFAULT_SYSTEM_ID = "";
constexpr std::string_view IMAGE_TRANSPORT_PARAMETER = "image_transport";
constexpr std::string_view DEFAULT_IMAGE_TRANSPORT = "raw";
constexpr std::string_view VOCABULARY_FILE_PARAMETER = "vocabulary_file";
constexpr std::string_view DEFAULT_VOCABULARY_FILE = "";
constexpr std::string_view SETTINGS_FILE_PARAMETER = "settings_file";
constexpr std::string_view DEFAULT_SETTINGS_FILE = "";
constexpr std::size_t ORB_IMU_SUB_QOS_DEPTH = 512;
constexpr std::size_t ORB_IMU_BUFFER_MAX_SAMPLES = 1024;
constexpr std::int64_t IMU_INTERVAL_GAP_WARN_NS = 30'000'000;
constexpr int IMU_DIAGNOSTIC_THROTTLE_MS = 5'000;

[[maybe_unused]] rclcpp::QoS BestEffortSensorQos(std::size_t depth)
{
  rclcpp::QoS qos{rclcpp::KeepLast(depth)};
  qos.best_effort();
  qos.durability_volatile();
  return qos;
}

rclcpp::QoS ReliableSensorQos(std::size_t depth)
{
  rclcpp::QoS qos{rclcpp::KeepLast(depth)};
  qos.reliable();
  qos.durability_volatile();
  return qos;
}
} // namespace

MonocularInertialSlamNode::MonocularInertialSlamNode(rclcpp::Node& node)
  : m_node(node),
    m_logger(std::make_unique<rclcpp::Logger>(node.get_logger())),
    m_imgSubscriber(std::make_unique<image_transport::Subscriber>())
{
  m_node.declare_parameter<std::string>(SYSTEM_ID_PARAMETER.data(), DEFAULT_SYSTEM_ID.data());
  m_node.declare_parameter<std::string>(IMAGE_TRANSPORT_PARAMETER.data(),
                                        DEFAULT_IMAGE_TRANSPORT.data());
  m_node.declare_parameter<std::string>(VOCABULARY_FILE_PARAMETER.data(),
                                        DEFAULT_VOCABULARY_FILE.data());
  m_node.declare_parameter<std::string>(SETTINGS_FILE_PARAMETER.data(),
                                        DEFAULT_SETTINGS_FILE.data());
}

MonocularInertialSlamNode::~MonocularInertialSlamNode() = default;

bool MonocularInertialSlamNode::Initialize()
{
  std::string systemId;
  if (!m_node.get_parameter(SYSTEM_ID_PARAMETER.data(), systemId) || systemId.empty())
  {
    RCLCPP_ERROR(*m_logger, "Missing or empty system ID parameter '%s'",
                 SYSTEM_ID_PARAMETER.data());
    return false;
  }

  std::string imageTopic = systemId;
  imageTopic.push_back('_');
  imageTopic.append(IMAGE_TOPIC);

  std::string imuTopic = systemId;
  imuTopic.push_back('_');
  imuTopic.append(IMU_TOPIC);

  std::string mapImageTopic = systemId;
  mapImageTopic.push_back('_');
  mapImageTopic.append(MAP_IMAGE_TOPIC);

  std::string pointCloudTopic = systemId;
  pointCloudTopic.push_back('_');
  pointCloudTopic.append(POINT_CLOUD_TOPIC);

  std::string poseTopic = systemId;
  poseTopic.push_back('_');
  poseTopic.append(POSE_TOPIC);

  RCLCPP_INFO(*m_logger, "System ID: %s", systemId.c_str());
  RCLCPP_INFO(*m_logger, "Image topic: %s", imageTopic.c_str());
  RCLCPP_INFO(*m_logger, "IMU topic: %s", imuTopic.c_str());
  RCLCPP_INFO(*m_logger, "Map image topic: %s", mapImageTopic.c_str());
  RCLCPP_INFO(*m_logger, "Point cloud topic: %s", pointCloudTopic.c_str());
  RCLCPP_INFO(*m_logger, "Pose topic: %s", poseTopic.c_str());

  std::string imageTransport;
  if (!m_node.get_parameter(IMAGE_TRANSPORT_PARAMETER.data(), imageTransport) ||
      imageTransport.empty())
  {
    imageTransport = std::string{DEFAULT_IMAGE_TRANSPORT};
    RCLCPP_WARN(*m_logger, "Image transport parameter '%s' missing or empty, defaulting to '%s'",
                IMAGE_TRANSPORT_PARAMETER.data(), imageTransport.c_str());
  }
  RCLCPP_INFO(*m_logger, "Image transport: %s", imageTransport.c_str());

  std::string vocabularyFile;
  if (!m_node.get_parameter(VOCABULARY_FILE_PARAMETER.data(), vocabularyFile) ||
      vocabularyFile.empty())
  {
    RCLCPP_ERROR(*m_logger, "Vocabulary parameter '%s' missing or empty",
                 VOCABULARY_FILE_PARAMETER.data());
    return false;
  }
  RCLCPP_INFO(*m_logger, "ORB_SLAM3 vocabulary file: %s", vocabularyFile.c_str());

  std::string settingsFile;
  if (!m_node.get_parameter(SETTINGS_FILE_PARAMETER.data(), settingsFile) || settingsFile.empty())
  {
    RCLCPP_ERROR(*m_logger, "Settings parameter '%s' missing or empty",
                 SETTINGS_FILE_PARAMETER.data());
    return false;
  }
  RCLCPP_INFO(*m_logger, "ORB_SLAM3 settings file: %s", settingsFile.c_str());

  *m_imgSubscriber = image_transport::create_subscription(
      &m_node, imageTopic, [this](const sensor_msgs::msg::Image::ConstSharedPtr& msg)
      { OnImage(msg); }, imageTransport, rclcpp::QoS{1}.get_rmw_qos_profile());

  m_imuSubscriber = m_node.create_subscription<sensor_msgs::msg::Imu>(
      imuTopic, ReliableSensorQos(ORB_IMU_SUB_QOS_DEPTH),
      std::bind(&MonocularInertialSlamNode::OnImu, this, std::placeholders::_1));

  m_monocularInertialSlam =
      std::make_unique<SLAM::MonocularInertialSlam>(m_node, pointCloudTopic, poseTopic);
  if (!m_monocularInertialSlam->Initialize(vocabularyFile, settingsFile))
  {
    RCLCPP_ERROR(*m_logger, "Failed to initialize monocular inertial SLAM");
    return false;
  }

  RCLCPP_INFO(*m_logger, "Started monocular inertial SLAM");

  return true;
}

void MonocularInertialSlamNode::Deinitialize()
{
  if (m_imgSubscriber)
    m_imgSubscriber->shutdown();

  m_imuSubscriber.reset();

  if (m_monocularInertialSlam)
  {
    m_monocularInertialSlam->Deinitialize();
    m_monocularInertialSlam.reset();
  }
}

void MonocularInertialSlamNode::OnImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  if (!msg)
    return;

  if (m_monocularInertialSlam)
  {
    const std::int64_t imageStampNs = StampToNanoseconds(msg->header.stamp);
    const std::vector<sensor_msgs::msg::Imu> imuSamples = TakeImuSamplesForImage(imageStampNs);
    m_monocularInertialSlam->ReceiveImageWithImuMeasurements(msg, imuSamples);
  }
}

void MonocularInertialSlamNode::OnImu(const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
{
  if (!msg)
    return;

  const std::int64_t stampNs = StampToNanoseconds(msg->header.stamp);
  std::lock_guard<std::mutex> lock(m_imuBufferMutex);
  const auto insertAt = std::upper_bound(m_imuBuffer.begin(), m_imuBuffer.end(), stampNs,
                                         [](std::int64_t lhs, const sensor_msgs::msg::Imu& rhs)
                                         { return lhs < StampToNanoseconds(rhs.header.stamp); });

  m_imuBuffer.insert(insertAt, *msg);

  while (m_imuBuffer.size() > ORB_IMU_BUFFER_MAX_SAMPLES)
  {
    m_imuBuffer.pop_front();
    ++m_imuBufferDropCount;
  }

  if (m_imuBufferDropCount > 0)
  {
    RCLCPP_WARN_THROTTLE(*m_logger, *m_node.get_clock(), IMU_DIAGNOSTIC_THROTTLE_MS,
                         "ORB IMU buffer overflow dropped %" PRIu64 " samples",
                         m_imuBufferDropCount);
  }
}

std::vector<sensor_msgs::msg::Imu> MonocularInertialSlamNode::TakeImuSamplesForImage(
    std::int64_t image_stamp_ns)
{
  std::vector<sensor_msgs::msg::Imu> imuSamples;
  std::lock_guard<std::mutex> lock(m_imuBufferMutex);

  if (m_imuBuffer.empty())
  {
    ++m_emptyImuBufferImageCount;
    RCLCPP_WARN_THROTTLE(*m_logger, *m_node.get_clock(), IMU_DIAGNOSTIC_THROTTLE_MS,
                         "ORB image arrived with an empty IMU buffer (count=%" PRIu64 ")",
                         m_emptyImuBufferImageCount);
  }

  if (m_lastImageStampNs.has_value())
  {
    const std::int64_t previousImageStampNs = *m_lastImageStampNs;
    for (const sensor_msgs::msg::Imu& imuMessage : m_imuBuffer)
    {
      const std::int64_t imuStampNs = StampToNanoseconds(imuMessage.header.stamp);
      if (previousImageStampNs < imuStampNs && imuStampNs <= image_stamp_ns)
        imuSamples.emplace_back(imuMessage);
    }

    DiagnoseImuBatch(imuSamples, previousImageStampNs, image_stamp_ns);
    PruneImuBuffer(previousImageStampNs);
  }

  m_lastImageStampNs = image_stamp_ns;
  return imuSamples;
}

void MonocularInertialSlamNode::PruneImuBuffer(std::int64_t previous_image_stamp_ns)
{
  while (!m_imuBuffer.empty() &&
         StampToNanoseconds(m_imuBuffer.front().header.stamp) < previous_image_stamp_ns)
  {
    m_imuBuffer.pop_front();
  }
}

void MonocularInertialSlamNode::DiagnoseImuBatch(
    const std::vector<sensor_msgs::msg::Imu>& imu_samples,
    std::int64_t previous_image_stamp_ns,
    std::int64_t image_stamp_ns)
{
  if (imu_samples.empty())
  {
    ++m_emptyImuIntervalCount;
    RCLCPP_WARN_THROTTLE(*m_logger, *m_node.get_clock(), IMU_DIAGNOSTIC_THROTTLE_MS,
                         "ORB image interval has no IMU samples: start_ns=%" PRId64
                         " end_ns=%" PRId64 " count=%" PRIu64,
                         previous_image_stamp_ns, image_stamp_ns, m_emptyImuIntervalCount);
    return;
  }

  const std::int64_t firstImuStampNs = StampToNanoseconds(imu_samples.front().header.stamp);
  if (firstImuStampNs - previous_image_stamp_ns > IMU_INTERVAL_GAP_WARN_NS)
  {
    RCLCPP_WARN_THROTTLE(*m_logger, *m_node.get_clock(), IMU_DIAGNOSTIC_THROTTLE_MS,
                         "ORB IMU interval starts late by %.3f ms",
                         static_cast<double>(firstImuStampNs - previous_image_stamp_ns) / 1.0e6);
  }

  std::int64_t previousImuStampNs = firstImuStampNs;
  for (std::size_t i = 1; i < imu_samples.size(); ++i)
  {
    const std::int64_t imuStampNs = StampToNanoseconds(imu_samples[i].header.stamp);
    if (imuStampNs - previousImuStampNs > IMU_INTERVAL_GAP_WARN_NS)
    {
      RCLCPP_WARN_THROTTLE(*m_logger, *m_node.get_clock(), IMU_DIAGNOSTIC_THROTTLE_MS,
                           "ORB IMU interval gap is %.3f ms",
                           static_cast<double>(imuStampNs - previousImuStampNs) / 1.0e6);
      break;
    }
    previousImuStampNs = imuStampNs;
  }
}

std::int64_t MonocularInertialSlamNode::StampToNanoseconds(
    const builtin_interfaces::msg::Time& stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1'000'000'000LL +
         static_cast<std::int64_t>(stamp.nanosec);
}
