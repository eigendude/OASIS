/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "apriltag/AprilTagVisualizer.h"
#include "slam/CameraModel.h"
#include "slam/MapViewRenderer.h"

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

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
  void OnDetections(const apriltag_msgs::msg::AprilTagDetectionArray::ConstSharedPtr& msg);
  bool OnImage(const sensor_msgs::msg::Image::ConstSharedPtr& imageMsg);
  void OnCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg);

  static Eigen::Isometry3f PoseMsgToIsometry(const geometry_msgs::msg::PoseStamped& poseMsg);
  static bool PointCloudToVector(const sensor_msgs::msg::PointCloud2& pointCloud,
                                 std::vector<Eigen::Vector3f>& worldPoints);

  rclcpp::Node& m_node;
  rclcpp::Logger m_logger;

  std::unique_ptr<image_transport::Publisher> m_mapImagePublisher;
  std::shared_ptr<image_transport::SubscriberFilter> m_imageSubscriber;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_poseSubscription;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_pointCloudSubscription;
  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr
      m_detectionsSubscription;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr m_cameraInfoSubscription;

  SLAM::CameraModel m_cameraModel;
  SLAM::MapViewRenderer m_renderer;
  AprilTagVisualizer m_aprilTagVisualizer;
  std::optional<Eigen::Isometry3f> m_cameraFromWorldTransform;

  cv::Mat m_imageBuffer;
  cv::Mat m_outputBuffer;

  double m_backgroundAlpha{0.25};
  std::string m_outputEncoding;

  cv::Mat m_backgroundImage;
  std_msgs::msg::Header m_backgroundHeader;
  std::mutex m_backgroundMutex;

  apriltag_msgs::msg::AprilTagDetectionArray::ConstSharedPtr m_latestDetectionsMsg;
  std::mutex m_detectionsMutex;
  bool m_warnedOutputEncoding{false};
};

} // namespace OASIS
