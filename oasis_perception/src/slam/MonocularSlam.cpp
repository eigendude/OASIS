/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "MonocularSlam.h"

#include <System.h>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/transport_hints.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/node.hpp>
#include <rcutils/logging_macros.h>
#include <sensor_msgs/image_encodings.hpp>

using namespace OASIS;
using namespace SLAM;

namespace
{
// TODO
constexpr const char* VOCABULARY_FILE =
    "/home/garrett/Documents/ros-ws/src/oasis/ros-ws/oasis-depends-humble/src/depends/orb-slam3/"
    "Vocabulary/ORBvoc.txt"; // TODO
constexpr const char* SETTINGS_FILE =
    "/home/garrett/Documents/ros-ws/src/oasis/oasis_perception/config/Webcam.yaml"; // TODO
} // namespace

MonocularSlam::MonocularSlam(std::shared_ptr<rclcpp::Node> node, const std::string& imageTopic)
  : m_logger(node->get_logger()),
    m_imgTransport(std::make_unique<image_transport::ImageTransport>(node)),
    m_imgSubscriber(std::make_unique<image_transport::Subscriber>()),
    m_slam(std::make_unique<ORB_SLAM3::System>(
        VOCABULARY_FILE, SETTINGS_FILE, ORB_SLAM3::System::MONOCULAR, false))
{
  RCLCPP_INFO(m_logger, "Image topic: %s", imageTopic.c_str());

  auto transportHints = image_transport::TransportHints(node.get(), "compressed");

  *m_imgSubscriber =
      m_imgTransport->subscribe(imageTopic, 1, &MonocularSlam::ReceiveImage, this, &transportHints);

  RCLCPP_INFO(m_logger, "Started monocular SLAM");
}

MonocularSlam::~MonocularSlam()
{
  // Stop all threads
  m_slam->Shutdown();

  //m_slam->SaveTrajectoryEuRoC("CameraTrajectory.txt");
  //m_slam->SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
}

void MonocularSlam::ReceiveImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(m_logger, "cv_bridge exception: %s", e.what());
    return;
  }

  double tframe = 0.0; // TODO

  // Pass the image to the SLAM system
  m_slam->TrackMonocular(cv_ptr->image, tframe);
}
