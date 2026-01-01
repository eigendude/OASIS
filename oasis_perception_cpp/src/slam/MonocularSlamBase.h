/*
 *  Copyright (C) 2021-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "CameraModel.h"

#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>
#include <System.h>
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

namespace rclcpp
{
class Logger;
class Node;
} // namespace rclcpp

namespace OASIS
{
namespace SLAM
{

class MonocularSlamBase
{
public:
  MonocularSlamBase(rclcpp::Node& node,
                    const std::string& pointCloudTopic,
                    const std::string& poseTopic);
  virtual ~MonocularSlamBase();

  void ReceiveImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

protected:
  bool InitializeSystem(const std::string& vocabularyFile,
                        const std::string& settingsFile,
                        ORB_SLAM3::System::eSensor sensorType);
  void DeinitializeSystem();

  virtual Eigen::Isometry3f TrackFrame(const cv::Mat& rgbImage, double timestamp) = 0;

  bool HasSlam() const;
  ORB_SLAM3::System* GetSlam();
  rclcpp::Logger& Logger();
  const rclcpp::Logger& Logger() const;

private:
  struct ImageProcessTask
  {
    std_msgs::msg::Header header;
    double timestamp = 0.0;
    cv_bridge::CvImageConstPtr inputImage;
  };

  // Threading functions
  void MapPublisherLoop();
  void StopMapPublisher();

  // Publishing functions
  void PublishPointCloud(const std_msgs::msg::Header& header,
                         const std::vector<Eigen::Vector3f>& worldPoints);

  // ROS parameters
  std::unique_ptr<rclcpp::Logger> m_logger;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pointCloudPublisher;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_posePublisher;

  // SLAM components
  CameraModel m_cameraModel;

  // ORB-SLAM3 system
  std::unique_ptr<ORB_SLAM3::System> m_slam;

  // Time synchronization
  std::optional<double> m_lastTimestamp;

  // Buffers
  std::vector<Eigen::Vector3f> m_worldPointBuffer;

  // Synchronization parameters
  std::deque<ImageProcessTask> m_renderQueue;
  std::mutex m_renderMutex;
  std::condition_variable m_renderCv;
  std::thread m_renderThread;
  std::atomic<bool> m_renderThreadRunning{false};
};

} // namespace SLAM
} // namespace OASIS
