/*
 *  Copyright (C) 2021-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "MonocularSlam.h"

#include "ros/RosUtils.h"

#include <algorithm>
#include <string>
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
  }
}

MonocularSlam::~MonocularSlam() = default;

bool MonocularSlam::Initialize(const std::string& vocabularyFile, const std::string& settingsFile)
{
  if (vocabularyFile.empty() || settingsFile.empty())
    return false;

  if (!LoadCameraModel(settingsFile, m_cameraModel, *m_logger))
    return false;

  m_mapViewRenderer.Initialize(m_cameraModel);

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

  m_mapViewRenderer.SetImageSize(rgbImage.cols, rgbImage.rows);

  // Pass the image to the SLAM system
  const Sophus::SE3f cameraPose = m_slam->TrackMonocular(rgbImage, timestamp);

  // Get SLAM properties
  const int trackingState = m_slam->GetTrackingState();
  const std::vector<ORB_SLAM3::MapPoint*> trackedMapPoints = m_slam->GetTrackedMapPoints();
  std::vector<ORB_SLAM3::MapPoint*> mapPoints;
  if (m_slam->GetAtlas() != nullptr)
    mapPoints = m_slam->GetAtlas()->GetAllMapPoints();

  RCLCPP_INFO(*m_logger, "Tracking state: %d, tracked points: %zu, map points: %zu", trackingState,
              trackedMapPoints.size(), mapPoints.size());

  if (m_mapImagePublisher && !mapPoints.empty())
  {
    if (m_mapViewRenderer.Render(cameraPose, mapPoints, trackedMapPoints, m_mapImageBuffer))
    {
      cv_bridge::CvImage output(header, sensor_msgs::image_encodings::RGB8, m_mapImageBuffer);
      m_mapImagePublisher->publish(output.toImageMsg());
    }
  }
}
