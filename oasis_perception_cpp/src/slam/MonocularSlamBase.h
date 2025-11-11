/*
 *  Copyright (C) 2021-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "CameraModel.h"
#include "MapViewRenderer.h"

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
#include <sophus/se3.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

namespace ORB_SLAM3
{
class MapPoint;
class System;
} // namespace ORB_SLAM3

namespace rclcpp
{
class Logger;
class Node;
} // namespace rclcpp

namespace cv
{
class Mat;
} // namespace cv

namespace OASIS
{
namespace SLAM
{

class MonocularSlamBase
{
public:
  MonocularSlamBase(rclcpp::Node& node, const std::string& mapImageTopic);
  virtual ~MonocularSlamBase();

  bool InitializeSystem(const std::string& vocabularyFile,
                        const std::string& settingsFile,
                        ORB_SLAM3::System::eSensor sensorType);
  void DeinitializeSystem();

  void ReceiveImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

protected:
  virtual Sophus::SE3f TrackFrame(const cv::Mat& rgbImage, double timestamp) = 0;
  virtual void OnInitialized();
  virtual void OnDeinitialized();
  virtual void OnPostTrack();

  bool HasSlam() const;
  ORB_SLAM3::System* GetSlam();
  rclcpp::Logger& Logger();
  const rclcpp::Logger& Logger() const;

private:
  struct MapRenderTask
  {
    std_msgs::msg::Header header;
    Sophus::SE3f cameraPose;
    std::vector<ORB_SLAM3::MapPoint*> mapPoints;
    int imageWidth = 0;
    int imageHeight = 0;
  };

  void MapPublisherLoop();
  void StopMapPublisher();

  // ROS parameters
  std::unique_ptr<rclcpp::Logger> m_logger;
  std::optional<image_transport::Publisher> m_mapImagePublisher;

  // SLAM components
  CameraModel m_cameraModel;
  MapViewRenderer m_mapViewRenderer;

  // ORB-SLAM3 system
  std::unique_ptr<ORB_SLAM3::System> m_slam;

  // SLAM properties
  cv::Mat m_mapImageBuffer;

  std::deque<MapRenderTask> m_renderQueue;
  std::mutex m_renderMutex;
  std::condition_variable m_renderCv;
  std::thread m_renderThread;
  std::atomic<bool> m_renderThreadRunning{false};
};

} // namespace SLAM
} // namespace OASIS

