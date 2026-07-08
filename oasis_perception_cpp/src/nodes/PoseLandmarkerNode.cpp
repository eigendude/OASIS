/*
 *  Copyright (C) 2025-2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "PoseLandmarkerNode.h"

#include "pose/PoseLandmarker.h"

#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/logging.hpp>
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
  const mediapipe_facade::PoseLandmarkerConfig config{
      .loggingName = m_node.get_name(),
  };

  if (!m_poseLandmarker->Initialize(config))
  {
    RCLCPP_ERROR(m_node.get_logger(), "PoseLandmarker initialization failed");
    return false;
  }

  RCLCPP_INFO(m_node.get_logger(), "PoseLandmarker initialized");

  const mediapipe_facade::PoseDetectionStubInput stubInput{
      .width = 0,
      .height = 0,
      .channelCount = 0,
      .encoding = "startup",
  };
  const mediapipe_facade::PoseDetectionStubResult stubResult =
      m_poseLandmarker->DetectStub(stubInput);
  if (!stubResult.success)
  {
    RCLCPP_ERROR(m_node.get_logger(), "PoseLandmarker facade startup check failed: %s",
                 stubResult.message.c_str());
    return false;
  }

  RCLCPP_INFO(m_node.get_logger(), "PoseLandmarker facade startup check succeeded: %s",
              stubResult.message.c_str());

  return true;
}

void PoseLandmarkerNode::Stop()
{
  // TODO
}

void PoseLandmarkerNode::OnImage(const sensor_msgs::msg::Image& imageMsg)
{
  // Convert the incoming image to an OpenCV image
  cv_bridge::CvImageConstPtr imagePtr;
  try
  {
    imagePtr = cv_bridge::toCvCopy(imageMsg, imageMsg.encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(m_node.get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  const mediapipe_facade::PoseDetectionStubInput stubInput{
      .width = imagePtr->image.cols,
      .height = imagePtr->image.rows,
      .channelCount = imagePtr->image.channels(),
      .encoding = imageMsg.encoding,
  };
  const mediapipe_facade::PoseDetectionStubResult stubResult =
      m_poseLandmarker->DetectStub(stubInput);
  if (!stubResult.success)
  {
    RCLCPP_ERROR(m_node.get_logger(), "PoseLandmarker facade detection failed: %s",
                 stubResult.message.c_str());
    return;
  }

  sensor_msgs::msg::Image::SharedPtr outMsg = imagePtr->toImageMsg();

  // Publish the image
  m_publisher.publish(*outMsg);
}
