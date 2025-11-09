/*
 *  Copyright (C) 2021-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "MonocularSlam.h"

#include "MapVisualizer.h"
#include "ros/RosUtils.h"

#include <algorithm>
#include <string>
#include <vector>

#include <System.h>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/image_encodings.hpp>

using namespace OASIS;
using namespace SLAM;

MonocularSlam::MonocularSlam(rclcpp::Node& node,
                             const std::string& mapTopic,
                             const std::string& mapImageTopic)
  : m_logger(std::make_unique<rclcpp::Logger>(node.get_logger()))
{
  m_visualizer = std::make_unique<MapVisualizer>(node, *m_logger, mapTopic, mapImageTopic);
}

MonocularSlam::~MonocularSlam() = default;

bool MonocularSlam::Initialize(const std::string& vocabularyFile, const std::string& settingsFile)
{
  if (vocabularyFile.empty() || settingsFile.empty())
    return false;

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

  if (m_visualizer)
    m_visualizer->Reset();
}

void MonocularSlam::ReceiveImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  if (!m_slam)
    return;

  const std_msgs::msg::Header& header = msg->header;
  const double timestamp = ROS::RosUtils::HeaderStampToSeconds(header);

  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(*m_logger, "cv_bridge exception: %s", e.what());
    return;
  }

  const cv::Mat& rgbImage = cv_ptr->image;

  if (rgbImage.type() != CV_8UC3)
  {
    RCLCPP_WARN(*m_logger,
                "Received image converted to unexpected type (type=%d channels=%d). Expected RGB8.",
                rgbImage.type(), rgbImage.channels());
  }

  // Pass the image to the SLAM system
  const Sophus::SE3f cameraPose = m_slam->TrackMonocular(rgbImage, timestamp);

  const int trackingState = m_slam->GetTrackingState();
  const std::vector<ORB_SLAM3::MapPoint*> trackedMapPoints = m_slam->GetTrackedMapPoints();
  const std::vector<cv::KeyPoint> trackedKeyPoints = m_slam->GetTrackedKeyPointsUn();

  std::size_t trackedMapPointCount = 0;
  for (const ORB_SLAM3::MapPoint* mapPoint : trackedMapPoints)
  {
    if (mapPoint != nullptr)
      ++trackedMapPointCount;
  }

  const Sophus::SE3f worldPose = cameraPose.inverse();
  const Eigen::Vector3f cameraPosition = worldPose.translation();
  const Eigen::Quaternionf cameraOrientation = worldPose.unit_quaternion();

  // clang-format off
  RCLCPP_INFO(*m_logger,
              "SLAM pose state=%d position=(%.3f, %.3f, %.3f) orientation=(%.4f, %.4f, %.4f, %.4f) tracked=%zu/%zu",
              trackingState,
              cameraPosition.x(),
              cameraPosition.y(),
              cameraPosition.z(),
              cameraOrientation.w(),
              cameraOrientation.x(),
              cameraOrientation.y(),
              cameraOrientation.z(),
              trackedMapPointCount,
              trackedKeyPoints.size());
  // clang-format on

  if (m_visualizer)
    m_visualizer->Publish(header, trackedMapPoints, cameraPosition, cameraOrientation);
}
