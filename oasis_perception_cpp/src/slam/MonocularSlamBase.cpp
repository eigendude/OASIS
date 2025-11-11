/*
 *  Copyright (C) 2021-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "MonocularSlamBase.h"

#include "ros/RosUtils.h"

#include <string>
#include <utility>
#include <vector>

#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/image_encodings.hpp>

using namespace OASIS;
using namespace SLAM;

MonocularSlamBase::MonocularSlamBase(rclcpp::Node& node, const std::string& mapImageTopic)
  : m_logger(std::make_unique<rclcpp::Logger>(node.get_logger()))
{
  rmw_qos_profile_t qos = rmw_qos_profile_sensor_data;
  qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;

  m_mapImagePublisher = image_transport::create_publisher(&node, mapImageTopic, qos);

  m_renderThreadRunning = true;
  m_renderThread = std::thread(&MonocularSlamBase::MapPublisherLoop, this);
}

MonocularSlamBase::~MonocularSlamBase()
{
  StopMapPublisher();
}

bool MonocularSlamBase::InitializeSystem(const std::string& vocabularyFile,
                                         const std::string& settingsFile,
                                         ORB_SLAM3::System::eSensor sensorType)
{
  if (vocabularyFile.empty() || settingsFile.empty())
    return false;

  if (!LoadCameraModel(settingsFile, m_cameraModel, Logger()))
    return false;

  m_mapViewRenderer.Initialize(m_cameraModel);

  m_slam = std::make_unique<ORB_SLAM3::System>(vocabularyFile, settingsFile, sensorType, false);

  return true;
}

void MonocularSlamBase::DeinitializeSystem()
{
  if (m_slam)
  {
    m_slam->Shutdown();
    m_slam.reset();
  }

  {
    std::lock_guard<std::mutex> lock(m_renderMutex);
    m_renderQueue.clear();
  }
}

void MonocularSlamBase::ReceiveImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  if (!HasSlam())
    return;

  const std_msgs::msg::Header& header = msg->header;
  const double timestamp = ROS::RosUtils::HeaderStampToSeconds(header);

  cv_bridge::CvImageConstPtr inputImage;
  try
  {
    inputImage = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(Logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  const cv::Mat& rgbImage = inputImage->image;

  const Eigen::Isometry3f cameraPose = TrackFrame(rgbImage, timestamp);

  ORB_SLAM3::System* slam = GetSlam();
  if (slam == nullptr)
    return;

  const int trackingState = slam->GetTrackingState();
  const std::vector<ORB_SLAM3::MapPoint*> trackedMapPoints = slam->GetTrackedMapPoints();
  std::vector<ORB_SLAM3::MapPoint*> mapPoints;
  if (slam->GetAtlas() != nullptr)
    mapPoints = slam->GetAtlas()->GetAllMapPoints();

  RCLCPP_INFO(Logger(), "Tracking state: %d, tracked points: %zu, map points: %zu", trackingState,
              trackedMapPoints.size(), mapPoints.size());

  if (m_mapImagePublisher && m_renderThreadRunning.load() && !mapPoints.empty())
  {
    MapRenderTask task;
    task.header = header;
    task.cameraPose = cameraPose;
    task.mapPoints = mapPoints;
    task.inputImage = std::move(inputImage);

    {
      std::lock_guard<std::mutex> lock(m_renderMutex);
      m_renderQueue.clear();
      m_renderQueue.emplace_back(std::move(task));
    }

    m_renderCv.notify_one();
  }

  OnPostTrack();
}

bool MonocularSlamBase::HasSlam() const
{
  return m_slam != nullptr;
}

ORB_SLAM3::System* MonocularSlamBase::GetSlam()
{
  return m_slam.get();
}

rclcpp::Logger& MonocularSlamBase::Logger()
{
  return *m_logger;
}

const rclcpp::Logger& MonocularSlamBase::Logger() const
{
  return *m_logger;
}

void MonocularSlamBase::MapPublisherLoop()
{
  while (true)
  {
    MapRenderTask task;
    {
      std::unique_lock<std::mutex> lock(m_renderMutex);
      m_renderCv.wait(lock,
                      [this] { return !m_renderThreadRunning.load() || !m_renderQueue.empty(); });

      if (!m_renderThreadRunning.load() && m_renderQueue.empty())
        break;

      task = std::move(m_renderQueue.front());
      m_renderQueue.pop_front();
    }

    if (m_mapViewRenderer.Render(task.cameraPose, task.mapPoints,
                                 const_cast<cv::Mat&>(task.inputImage->image)))
    {
      cv_bridge::CvImage output(task.header, sensor_msgs::image_encodings::RGB8,
                                task.inputImage->image);
      m_mapImagePublisher->publish(output.toImageMsg());
    }
  }
}

void MonocularSlamBase::StopMapPublisher()
{
  m_renderThreadRunning = false;

  if (m_renderThread.joinable())
  {
    m_renderCv.notify_all();
    m_renderThread.join();
  }

  {
    std::lock_guard<std::mutex> lock(m_renderMutex);
    m_renderQueue.clear();
  }
}
