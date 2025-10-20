/*
 *  Copyright (C) 2021-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "MonocularSlam.h"

#include "ros/RosUtils.h"

#include <cstddef>
#include <filesystem>
#include <stdexcept>

#include <System.h>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/transport_hints.hpp>
#include <Eigen/Geometry>
#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sophus/se3.hpp>

using namespace OASIS;
using namespace SLAM;

MonocularSlam::MonocularSlam(rclcpp::Node& node)
  : m_logger(std::make_unique<rclcpp::Logger>(node.get_logger()))
{
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
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(*m_logger, "cv_bridge exception: %s", e.what());
    return;
  }

  // Pass the image to the SLAM system
  const Sophus::SE3f cameraPose = m_slam->TrackMonocular(cv_ptr->image, timestamp);

  const int trackingState = m_slam->GetTrackingState();
  const auto trackedMapPoints = m_slam->GetTrackedMapPoints();
  const auto trackedKeyPoints = m_slam->GetTrackedKeyPointsUn();

  std::size_t trackedMapPointCount = 0;
  for (const auto* mapPoint : trackedMapPoints)
  {
    if (mapPoint != nullptr)
      ++trackedMapPointCount;
  }

  const auto& translation = cameraPose.translation();
  const Eigen::Quaternionf quaternion = cameraPose.unit_quaternion();

  RCLCPP_INFO(*m_logger,
              "SLAM pose timestamp=%.9f state=%d position=(%.3f, %.3f, %.3f) orientation=(%.4f, %.4f, %.4f, %.4f) tracked=%zu/%zu",
              timestamp,
              trackingState,
              translation.x(),
              translation.y(),
              translation.z(),
              quaternion.w(),
              quaternion.x(),
              quaternion.y(),
              quaternion.z(),
              trackedMapPointCount,
              trackedKeyPoints.size());
}
