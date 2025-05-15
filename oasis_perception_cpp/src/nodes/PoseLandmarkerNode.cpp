/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "PoseLandmarkerNode.h"

#include "pose/PoseLandmarker.h"

// TODO
// Note: All this commented stuff came from my kinect2_downscaler node in the
// oasis_kinect2 package. Needs to be updated for the new pose landmarker

/*
#include <algorithm>

#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>
*/
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace oasis_perception;

namespace
{
// TODO: Any constants go here
/*
constexpr unsigned int DOWNSCALED_maxWidth = 640;
constexpr unsigned int DOWNSCALED_maxHeight = 480;
*/
} // namespace

PoseLandmarkerNode::PoseLandmarkerNode(rclcpp::Node& node)
  : m_node(node), m_poseLandmarker(std::make_unique<PoseLandmarker>())
{
  // TODO
  //m_node.declare_parameter<std::string>("base_name", K2_DEFAULT_NS);
}

PoseLandmarkerNode::~PoseLandmarkerNode() = default;

bool PoseLandmarkerNode::start()
{
  // TODO
  /*
  std::string base_name;

  m_node.get_parameter_or("base_name", base_name, std::string(K2_DEFAULT_NS));

  m_publisher = image_transport::create_publisher(&m_node, base_name + K2_TOPIC_SD + K2_TOPIC_IMAGE_COLOR);
  m_subscriber = image_transport::create_subscription(&m_node, base_name + K2_TOPIC_QHD + K2_TOPIC_IMAGE_COLOR,
    [this](const auto& msg)
    {
      OnImage(msg);
    },
    "raw");
  */

  return true;
}

void PoseLandmarkerNode::stop()
{
  // TODO
  /*
  m_subscriber.shutdown();
  m_publisher.shutdown();
  */
}

void PoseLandmarkerNode::OnImage(const std::shared_ptr<const sensor_msgs::msg::Image>& msg)
{
  // Convert the incoming image to an OpenCV image
  cv_bridge::CvImageConstPtr imagePtr;
  try
  {
    imagePtr = cv_bridge::toCvShare(msg, msg->encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(m_node.get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  auto outMsg = m_poseLandmarker->OnImage(imagePtr);

  // Publish the image
  m_publisher.publish(*outMsg);
}
