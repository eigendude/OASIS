/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "PoseLandmarkerNode.h"

#include "pose/PoseLandmarker.h"

#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace oasis_perception;

PoseLandmarkerNode::PoseLandmarkerNode(rclcpp::Node& node)
  : m_node(node), m_poseLandmarker(std::make_unique<PoseLandmarker>())
{
}

PoseLandmarkerNode::~PoseLandmarkerNode() = default;

bool PoseLandmarkerNode::Start()
{
  if (!m_poseLandmarker->Initialize(m_node.get_name()))
  {
    RCLCPP_ERROR(m_node.get_logger(), "PoseLandmarker initialization failed");
    return false;
  }

  RCLCPP_INFO(m_node.get_logger(), "PoseLandmarker initialized");

  return true;
}

void PoseLandmarkerNode::Stop()
{
  // TODO
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
