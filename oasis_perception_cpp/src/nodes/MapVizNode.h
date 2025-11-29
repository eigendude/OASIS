/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "slam/CameraModel.h"
#include "slam/MapViewRenderer.h"

#include <memory>
#include <optional>
#include <string>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace OASIS
{

class MapVizNode
{
public:
  explicit MapVizNode(rclcpp::Node& node);
  ~MapVizNode();

  bool Initialize();
  void Deinitialize();

private:
  void OnPose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg);
  void OnPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);

  static Eigen::Isometry3f PoseMsgToIsometry(const geometry_msgs::msg::PoseStamped& poseMsg);
  static bool PointCloudToVector(const sensor_msgs::msg::PointCloud2& pointCloud,
                                 std::vector<Eigen::Vector3f>& worldPoints);

  rclcpp::Node& m_node;
  rclcpp::Logger m_logger;

  std::unique_ptr<image_transport::Publisher> m_mapImagePublisher;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_poseSubscription;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_pointCloudSubscription;

  SLAM::CameraModel m_cameraModel;
  SLAM::MapViewRenderer m_renderer;
  std::optional<Eigen::Isometry3f> m_cameraFromWorldTransform;

  cv::Mat m_imageBuffer;
};

} // namespace OASIS
