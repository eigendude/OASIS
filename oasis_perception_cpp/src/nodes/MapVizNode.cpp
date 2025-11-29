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
#include <array>
#include <cmath>
#include <functional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <apriltag_msgs/msg/april_tag_detection.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace OASIS
{
namespace
{
// ROS topics
constexpr std::string_view POSE_TOPIC_SUFFIX = "slam_pose";
constexpr std::string_view POINT_CLOUD_TOPIC_SUFFIX = "slam_point_cloud";
constexpr std::string_view MAP_IMAGE_TOPIC_SUFFIX = "slam_map_image";
constexpr std::string_view APRILTAG_TOPIC_SUFFIX = "apriltags";
constexpr std::string_view CAMERA_INFO_TOPIC_SUFFIX = "camera_info";

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

  std::string aprilTagTopic = systemId;
  aprilTagTopic.push_back('_');
  aprilTagTopic.append(APRILTAG_TOPIC_SUFFIX);

  std::string cameraInfoTopic = systemId;
  cameraInfoTopic.push_back('_');
  cameraInfoTopic.append(CAMERA_INFO_TOPIC_SUFFIX);

  RCLCPP_INFO(m_logger, "System ID: %s", systemId.c_str());
  RCLCPP_INFO(m_logger, "Pose topic: %s", poseTopic.c_str());
  RCLCPP_INFO(m_logger, "Point cloud topic: %s", pointCloudTopic.c_str());
  RCLCPP_INFO(m_logger, "Map image topic: %s", mapImageTopic.c_str());
  RCLCPP_INFO(m_logger, "AprilTag topic: %s", aprilTagTopic.c_str());
  RCLCPP_INFO(m_logger, "Camera info topic: %s", cameraInfoTopic.c_str());
  RCLCPP_INFO(m_logger, "Settings file: %s", settingsFile.c_str());

  *m_mapImagePublisher = image_transport::create_publisher(&m_node, mapImageTopic);

  m_poseSubscription = m_node.create_subscription<geometry_msgs::msg::PoseStamped>(
      poseTopic, {1},
      [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg) { OnPose(msg); });

  m_pointCloudSubscription = m_node.create_subscription<sensor_msgs::msg::PointCloud2>(
      pointCloudTopic, {1},
      [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) { OnPointCloud(msg); });

  m_aprilTagSubscription = m_node.create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
      aprilTagTopic, {1},
      [this](const apriltag_msgs::msg::AprilTagDetectionArray::ConstSharedPtr& msg)
      { OnAprilTags(msg); });

  m_cameraInfoSubscription = m_node.create_subscription<sensor_msgs::msg::CameraInfo>(
      cameraInfoTopic, {1},
      [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg) { OnCameraInfo(msg); });

  return true;
}

void MapVizNode::Deinitialize()
{
  m_cameraInfoSubscription.reset();
  m_aprilTagSubscription.reset();
  m_pointCloudSubscription.reset();
  m_poseSubscription.reset();
  m_mapImagePublisher->shutdown();
}

void MapVizNode::OnPose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg)
{
  if (msg == nullptr)
    return;

  m_cameraFromWorldTransform = PoseMsgToIsometry(*msg);
}

void MapVizNode::OnPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
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

  std::vector<SLAM::MapViewRenderer::OverlayQuadrilateral> overlays;
  if (m_cameraModel.width > 0 && m_cameraModel.height > 0)
  {
    std::scoped_lock lock(m_aprilTagMutex);
    overlays.reserve(m_latestAprilTagCorners.size());

    for (const auto& aprilTagCorners : m_latestAprilTagCorners)
    {
      SLAM::MapViewRenderer::OverlayQuadrilateral overlay;
      overlay.pixelCorners = aprilTagCorners;
      overlays.emplace_back(overlay);
    }
  }

  if (!m_renderer.Render(*m_cameraFromWorldTransform, worldPoints, overlays, m_imageBuffer))
    return;

  cv_bridge::CvImage output(msg->header, sensor_msgs::image_encodings::RGB8, m_imageBuffer);
  m_mapImagePublisher->publish(output.toImageMsg());
}

void MapVizNode::OnAprilTags(const apriltag_msgs::msg::AprilTagDetectionArray::ConstSharedPtr& msg)
{
  if (msg == nullptr)
    return;

  std::scoped_lock lock(m_aprilTagMutex);
  m_latestAprilTagCorners.clear();
  m_latestAprilTagCorners.reserve(msg->detections.size());

  for (const apriltag_msgs::msg::AprilTagDetection& detection : msg->detections)
  {
    if (detection.corners.size() < 4)
      continue;

    std::array<cv::Point2f, 4> corners;
    for (std::size_t index = 0; index < 4; ++index)
    {
      corners[index].x = static_cast<float>(detection.corners[index].x);
      corners[index].y = static_cast<float>(detection.corners[index].y);
    }

    m_latestAprilTagCorners.emplace_back(corners);
  }
}

void MapVizNode::OnCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg)
{
  if (msg == nullptr)
    return;

  SLAM::CameraModel updatedModel = m_cameraModel;

  if (msg->k.size() >= 6)
  {
    updatedModel.fx = static_cast<float>(msg->k[0]);
    updatedModel.fy = static_cast<float>(msg->k[4]);
    updatedModel.cx = static_cast<float>(msg->k[2]);
    updatedModel.cy = static_cast<float>(msg->k[5]);
  }

  if (msg->width > 0)
    updatedModel.width = msg->width;

  if (msg->height > 0)
    updatedModel.height = msg->height;

  if (updatedModel.width == m_cameraModel.width && updatedModel.height == m_cameraModel.height &&
      updatedModel.fx == m_cameraModel.fx && updatedModel.fy == m_cameraModel.fy &&
      updatedModel.cx == m_cameraModel.cx && updatedModel.cy == m_cameraModel.cy)
  {
    return;
  }

  m_cameraModel = updatedModel;
  m_renderer.Initialize(m_cameraModel);
}

Eigen::Isometry3f MapVizNode::PoseMsgToIsometry(const geometry_msgs::msg::PoseStamped& poseMsg)
{
  Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();

  const geometry_msgs::msg::Point& position = poseMsg.pose.position;
  transform.translation() =
      Eigen::Vector3f(static_cast<float>(position.x), static_cast<float>(position.y),
                      static_cast<float>(position.z));

  const geometry_msgs::msg::Quaternion& orientation = poseMsg.pose.orientation;
  Eigen::Quaternionf quaternion(
      static_cast<float>(orientation.w), static_cast<float>(orientation.x),
      static_cast<float>(orientation.y), static_cast<float>(orientation.z));
  quaternion.normalize();
  transform.linear() = quaternion.toRotationMatrix();

  return transform;
}

bool MapVizNode::PointCloudToVector(const sensor_msgs::msg::PointCloud2& pointCloud,
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
