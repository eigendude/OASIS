/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "MonocularInertialSlamNode.h"

#include "slam/MonocularInertialSlam.h"

#include <functional>
#include <memory>
#include <string>
#include <string_view>

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

  m_imuSubscriber = m_node.create_subscription<oasis_msgs::msg::I2CImu>(
      imuTopic, rclcpp::QoS{1},
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
  if (m_monocularInertialSlam)
    m_monocularInertialSlam->ReceiveImage(msg);
}

void MonocularInertialSlamNode::OnImu(const oasis_msgs::msg::I2CImu::ConstSharedPtr& msg)
{
  if (m_monocularInertialSlam)
    m_monocularInertialSlam->ImuCallback(msg);
}
