/*
 *  Copyright (C) 2021-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "MonocularSlam.h"

#include "ros/RosUtils.h"

#include <string>
#include <utility>
#include <vector>

#include <System.h>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/image_encodings.hpp>

using namespace OASIS;
using namespace SLAM;

MonocularSlam::MonocularSlam(rclcpp::Node& node, const std::string& mapImageTopic)
  : m_logger(std::make_unique<rclcpp::Logger>(node.get_logger()))
{
  if (!mapImageTopic.empty())
  {
    rmw_qos_profile_t qos = rmw_qos_profile_sensor_data;
    qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;

    m_mapImagePublisher = image_transport::create_publisher(&node, mapImageTopic, qos);

    m_renderThreadRunning = true;
    m_renderThread = std::thread(&MonocularSlam::MapPublisherLoop, this);
  }
}

MonocularSlam::~MonocularSlam()
{
  StopMapPublisher();
}

bool MonocularSlam::Initialize(const std::string& vocabularyFile, const std::string& settingsFile)
{
  if (vocabularyFile.empty() || settingsFile.empty())
    return false;

  if (!LoadCameraModel(settingsFile, m_cameraModel, *m_logger))
    return false;

  m_mapViewRenderer.SetCameraModel(m_cameraModel);
  m_mapViewRenderer.SetImageSize(m_cameraModel.width, m_cameraModel.height);

  m_slam = std::make_unique<ORB_SLAM3::System>(vocabularyFile, settingsFile,
                                               ORB_SLAM3::System::MONOCULAR, false);

  return true;
}

void MonocularSlam::Deinitialize()
{
  if (m_slam)
  {
    // Stop all threads
    m_slam->Shutdown();

    //m_slam->SaveTrajectoryEuRoC("CameraTrajectory.txt");
    //m_slam->SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");

    m_slam.reset();
  }

  {
    std::lock_guard<std::mutex> lock(m_renderMutex);
    m_renderQueue.clear();
  }
}

void MonocularSlam::ReceiveImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  if (!m_slam)
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
    RCLCPP_ERROR(*m_logger, "cv_bridge exception: %s", e.what());
    return;
  }

  const cv::Mat& rgbImage = inputImage->image;

  // Pass the image to the SLAM system
  const Sophus::SE3f cameraPose = m_slam->TrackMonocular(rgbImage, timestamp);

  const int trackingState = m_slam->GetTrackingState();
  const std::vector<ORB_SLAM3::MapPoint*> trackedMapPoints = m_slam->GetTrackedMapPoints();
  std::vector<ORB_SLAM3::MapPoint*> mapPoints;
  if (m_slam->GetAtlas() != nullptr)
    mapPoints = m_slam->GetAtlas()->GetAllMapPoints();

  RCLCPP_INFO(*m_logger, "Tracking state: %d, tracked points: %zu, map points: %zu", trackingState,
              trackedMapPoints.size(), mapPoints.size());

  if (m_mapImagePublisher && m_renderThreadRunning.load() && !mapPoints.empty())
  {
    MapRenderTask task;
    task.header = header;
    task.cameraPose = cameraPose;
    task.mapPoints = mapPoints;
    task.imageWidth = rgbImage.cols;
    task.imageHeight = rgbImage.rows;

    {
      std::lock_guard<std::mutex> lock(m_renderMutex);
      m_renderQueue.clear();
      m_renderQueue.emplace_back(std::move(task));
    }

    m_renderCv.notify_one();
  }
}

void MonocularSlam::MapPublisherLoop()
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

    m_mapViewRenderer.SetImageSize(task.imageWidth, task.imageHeight);
    if (m_mapViewRenderer.Render(task.cameraPose, task.mapPoints, m_mapImageBuffer))
    {
      cv_bridge::CvImage output(task.header, sensor_msgs::image_encodings::RGB8, m_mapImageBuffer);
      m_mapImagePublisher->publish(output.toImageMsg());
    }
  }
}

void MonocularSlam::StopMapPublisher()
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
