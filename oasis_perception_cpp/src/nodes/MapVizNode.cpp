/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "nodes/MapVizNode.h"

#include "slam/CameraModel.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

using geometry_msgs::msg::PoseStamped;
using sensor_msgs::msg::PointCloud2;

namespace OASIS
{
namespace
{
// ROS topics
constexpr std::string_view POSE_TOPIC_SUFFIX = "slam_pose";
constexpr std::string_view POINT_CLOUD_TOPIC_SUFFIX = "slam_point_cloud";
constexpr std::string_view MAP_IMAGE_TOPIC_SUFFIX = "slam_map_image";

// ROS parameters
constexpr std::string_view SYSTEM_ID_PARAMETER = "system_id";
constexpr std::string_view DEFAULT_SYSTEM_ID = "";

constexpr std::string_view SETTINGS_FILE_PARAMETER = "settings_file";
constexpr std::string_view DEFAULT_SETTINGS_FILE = "";
} // namespace

MapVizNode::MapVizNode(rclcpp::Node& node)
  : m_node(node),
    m_logger(node.get_logger()),
    m_mapImagePublisher(std::make_unique<image_transport::Publisher>())
{
  m_node.declare_parameter<std::string>(SYSTEM_ID_PARAMETER.data(), DEFAULT_SYSTEM_ID.data());
  m_node.declare_parameter<std::string>(SETTINGS_FILE_PARAMETER.data(),
                                        DEFAULT_SETTINGS_FILE.data());
}

MapVizNode::~MapVizNode() = default;

bool MapVizNode::Initialize()
{
  std::string systemId;
  if (!m_node.get_parameter(SYSTEM_ID_PARAMETER.data(), systemId) || systemId.empty())
  {
    RCLCPP_ERROR(m_logger, "Missing or empty system ID parameter '%s'", SYSTEM_ID_PARAMETER.data());
    return false;
  }

  std::string settingsFile;
  if (!m_node.get_parameter(SETTINGS_FILE_PARAMETER.data(), settingsFile) || settingsFile.empty())
  {
    RCLCPP_ERROR(m_logger, "Missing or empty settings file parameter '%s'",
                 SETTINGS_FILE_PARAMETER.data());
    return false;
  }

  if (!SLAM::LoadCameraModel(settingsFile, m_cameraModel, m_logger))
  {
    RCLCPP_ERROR(m_logger, "Failed to load camera model from settings file: %s",
                 settingsFile.c_str());
    return false;
  }

  m_renderer.Initialize(m_cameraModel);

  std::string poseTopic = systemId;
  poseTopic.push_back('_');
  poseTopic.append(POSE_TOPIC_SUFFIX);

  std::string pointCloudTopic = systemId;
  pointCloudTopic.push_back('_');
  pointCloudTopic.append(POINT_CLOUD_TOPIC_SUFFIX);

  std::string mapImageTopic = systemId;
  mapImageTopic.push_back('_');
  mapImageTopic.append(MAP_IMAGE_TOPIC_SUFFIX);

  RCLCPP_INFO(m_logger, "System ID: %s", systemId.c_str());
  RCLCPP_INFO(m_logger, "Pose topic: %s", poseTopic.c_str());
  RCLCPP_INFO(m_logger, "Point cloud topic: %s", pointCloudTopic.c_str());
  RCLCPP_INFO(m_logger, "Map image topic: %s", mapImageTopic.c_str());
  RCLCPP_INFO(m_logger, "Settings file: %s", settingsFile.c_str());

  rclcpp::QoS sensorQos = rclcpp::SensorDataQoS();
  sensorQos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  sensorQos.keep_last(1);

  *m_mapImagePublisher =
      image_transport::create_publisher(&m_node, mapImageTopic, sensorQos.get_rmw_qos_profile());

  m_poseSubscription = m_node.create_subscription<PoseStamped>(
      poseTopic, sensorQos, std::bind(&MapVizNode::OnPose, this, std::placeholders::_1));

  m_pointCloudSubscription = m_node.create_subscription<PointCloud2>(
      pointCloudTopic, sensorQos,
      std::bind(&MapVizNode::OnPointCloud, this, std::placeholders::_1));

  return true;
}

void MapVizNode::Deinitialize()
{
  m_pointCloudSubscription.reset();
  m_poseSubscription.reset();
  m_mapImagePublisher->shutdown();
}

void MapVizNode::OnPose(const PoseStamped::ConstSharedPtr& msg)
{
  if (msg == nullptr)
    return;

  m_cameraFromWorldTransform = PoseMsgToIsometry(*msg);
}

void MapVizNode::OnPointCloud(const PointCloud2::ConstSharedPtr& msg)
{
  if (!m_mapImagePublisher || m_mapImagePublisher->getNumSubscribers() == 0)
    return;

  if (!m_cameraFromWorldTransform)
  {
    RCLCPP_WARN_THROTTLE(m_logger, *m_node.get_clock(), 5000,
                         "Skipping map viz: waiting for slam pose");
    return;
  }

  std::vector<Eigen::Vector3f> worldPoints;
  if (!PointCloudToVector(*msg, worldPoints) || worldPoints.empty())
    return;

  if (!m_renderer.Render(*m_cameraFromWorldTransform, worldPoints, m_imageBuffer))
    return;

  cv_bridge::CvImage output(msg->header, sensor_msgs::image_encodings::RGB8, m_imageBuffer);
  m_mapImagePublisher->publish(output.toImageMsg());
}

Eigen::Isometry3f MapVizNode::PoseMsgToIsometry(const PoseStamped& poseMsg)
{
  Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();

  const auto& position = poseMsg.pose.position;
  transform.translation() =
      Eigen::Vector3f(static_cast<float>(position.x), static_cast<float>(position.y),
                      static_cast<float>(position.z));

  const auto& orientation = poseMsg.pose.orientation;
  Eigen::Quaternionf quaternion(
      static_cast<float>(orientation.w), static_cast<float>(orientation.x),
      static_cast<float>(orientation.y), static_cast<float>(orientation.z));
  quaternion.normalize();
  transform.linear() = quaternion.toRotationMatrix();

  return transform;
}

bool MapVizNode::PointCloudToVector(const PointCloud2& pointCloud,
                                    std::vector<Eigen::Vector3f>& worldPoints)
{
  if (pointCloud.width == 0 || pointCloud.height == 0)
    return false;

  sensor_msgs::PointCloud2ConstIterator<float> iterX(pointCloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iterY(pointCloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iterZ(pointCloud, "z");

  const std::size_t pointCount = static_cast<std::size_t>(pointCloud.width) * pointCloud.height;
  worldPoints.clear();
  worldPoints.reserve(pointCount);

  for (std::size_t index = 0; index < pointCount; ++index, ++iterX, ++iterY, ++iterZ)
  {
    const float x = *iterX;
    const float y = *iterY;
    const float z = *iterZ;

    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
      continue;

    worldPoints.emplace_back(x, y, z);
  }

  return true;
}

} // namespace OASIS
