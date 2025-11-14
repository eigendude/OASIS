/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "nodes/MeshViewerNode.h"

#include "pointcloud/MeshRenderer.h"

#include <functional>
#include <memory>
#include <string>
#include <string_view>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>

using sensor_msgs::msg::Image;
using sensor_msgs::msg::PointCloud2;

namespace OASIS
{
namespace
{
// ROS topics
constexpr std::string_view INPUT_TOPIC_SUFFIX = "slam_point_cloud";
constexpr std::string_view OUTPUT_TOPIC_SUFFIX = "slam_mesh_image";

// ROS parameters
constexpr std::string_view SYSTEM_ID_PARAMETER = "system_id";
constexpr std::string_view DEFAULT_SYSTEM_ID = "";

constexpr std::string_view VOXEL_LEAF_SIZE_PARAMETER = "voxel_leaf_size";
constexpr double DEFAULT_VOXEL_LEAF_SIZE = 0.05;

constexpr std::string_view NORMAL_SEARCH_RADIUS_PARAMETER = "normal_search_radius";
constexpr double DEFAULT_NORMAL_SEARCH_RADIUS = 0.1;

constexpr std::string_view TRIANGULATION_SEARCH_RADIUS_PARAMETER = "triangulation_search_radius";
constexpr double DEFAULT_TRIANGULATION_SEARCH_RADIUS = 0.2;

} // namespace

MeshViewerNode::MeshViewerNode(rclcpp::Node& node)
  : m_node(node),
    m_voxelLeafSize(DEFAULT_VOXEL_LEAF_SIZE),
    m_normalSearchRadius(DEFAULT_NORMAL_SEARCH_RADIUS),
    m_triangulationSearchRadius(DEFAULT_TRIANGULATION_SEARCH_RADIUS)
{
  m_node.declare_parameter<std::string>(SYSTEM_ID_PARAMETER.data(), DEFAULT_SYSTEM_ID.data());
  m_node.declare_parameter<double>(VOXEL_LEAF_SIZE_PARAMETER.data(), m_voxelLeafSize);
  m_node.declare_parameter<double>(NORMAL_SEARCH_RADIUS_PARAMETER.data(), m_normalSearchRadius);
  m_node.declare_parameter<double>(TRIANGULATION_SEARCH_RADIUS_PARAMETER.data(),
                                   m_triangulationSearchRadius);
}

MeshViewerNode::~MeshViewerNode() = default;

bool MeshViewerNode::Initialize()
{
  std::string systemId;
  if (!m_node.get_parameter(SYSTEM_ID_PARAMETER.data(), systemId))
  {
    RCLCPP_ERROR(m_node.get_logger(), "Missing system ID parameter '%s'",
                 SYSTEM_ID_PARAMETER.data());
    return false;
  }

  if (systemId.empty())
  {
    RCLCPP_ERROR(m_node.get_logger(), "System ID parameter '%s' is empty",
                 SYSTEM_ID_PARAMETER.data());
    return false;
  }

  if (!m_node.get_parameter(VOXEL_LEAF_SIZE_PARAMETER.data(), m_voxelLeafSize))
    m_voxelLeafSize = DEFAULT_VOXEL_LEAF_SIZE;
  if (!m_node.get_parameter(NORMAL_SEARCH_RADIUS_PARAMETER.data(), m_normalSearchRadius))
    m_normalSearchRadius = DEFAULT_NORMAL_SEARCH_RADIUS;
  if (!m_node.get_parameter(TRIANGULATION_SEARCH_RADIUS_PARAMETER.data(),
                            m_triangulationSearchRadius))
    m_triangulationSearchRadius = DEFAULT_TRIANGULATION_SEARCH_RADIUS;

  std::string pointCloudTopic = systemId;
  pointCloudTopic.push_back('_');
  pointCloudTopic.append(INPUT_TOPIC_SUFFIX);

  std::string meshImageTopic = systemId;
  meshImageTopic.push_back('_');
  meshImageTopic.append(OUTPUT_TOPIC_SUFFIX);

  RCLCPP_INFO(m_node.get_logger(), "System ID: %s", systemId.c_str());
  RCLCPP_INFO(m_node.get_logger(), "Point cloud topic: %s", pointCloudTopic.c_str());
  RCLCPP_INFO(m_node.get_logger(), "Mesh image topic: %s", meshImageTopic.c_str());
  RCLCPP_INFO(m_node.get_logger(), "Voxel leaf size: %.3f", m_voxelLeafSize);
  RCLCPP_INFO(m_node.get_logger(), "Normal search radius: %.3f", m_normalSearchRadius);
  RCLCPP_INFO(m_node.get_logger(), "Triangulation search radius: %.3f",
              m_triangulationSearchRadius);

  m_meshRenderer = std::make_unique<MeshRenderer>(
      m_node.get_logger(), m_voxelLeafSize, m_normalSearchRadius, m_triangulationSearchRadius);

  m_meshImagePublisher =
      m_node.create_publisher<Image>(meshImageTopic, rclcpp::SensorDataQoS().keep_last(1));

  m_pointCloudSubscription = m_node.create_subscription<PointCloud2>(
      pointCloudTopic, rclcpp::SensorDataQoS(),
      std::bind(&MeshViewerNode::OnPointCloud, this, std::placeholders::_1));

  return true;
}

void MeshViewerNode::Deinitialize()
{
  m_meshRenderer.reset();
  m_pointCloudSubscription.reset();
  m_meshImagePublisher.reset();
}

void MeshViewerNode::OnPointCloud(const PointCloud2::ConstSharedPtr& msg)
{
  if (!m_meshImagePublisher || !m_meshRenderer)
    return;

  auto imageMsg = m_meshRenderer->Render(*msg);
  if (!imageMsg)
    return;

  m_meshImagePublisher->publish(*imageMsg);
}
} // namespace OASIS
