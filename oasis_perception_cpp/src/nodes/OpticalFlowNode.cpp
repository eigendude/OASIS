/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "OpticalFlowNode.h"

#include "video/OpticalFlow.h"

#include <functional>
#include <string>

#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/image_encodings.hpp>

using namespace OASIS;
using namespace ROS;

namespace
{
// Published topics
constexpr const char* FLOW_TOPIC = "flow";

// Subscribed topics
constexpr const char* IMAGE_TOPIC = "image";

// Parameters
constexpr const char* SYSTEM_ID_PARAMETER = "system_id";
constexpr const char* DEFAULT_SYSTEM_ID = "";
} // namespace

OpticalFlowNode::OpticalFlowNode(rclcpp::Node& node)
  : m_node(node),
    m_logger(node.get_logger()),
    m_flowPublisher(std::make_unique<image_transport::Publisher>()),
    m_imgSubscriber(std::make_unique<image_transport::Subscriber>()),
    m_opticalFlow(std::make_unique<VIDEO::OpticalFlow>())
{
  m_node.declare_parameter<std::string>(SYSTEM_ID_PARAMETER, DEFAULT_SYSTEM_ID);
}

OpticalFlowNode::~OpticalFlowNode() = default;

bool OpticalFlowNode::Initialize()
{
  RCLCPP_INFO(m_node.get_logger(), "Starting optical flow...");

  std::string systemId;
  if (!m_node.get_parameter(SYSTEM_ID_PARAMETER, systemId))
  {
    RCLCPP_ERROR(m_node.get_logger(), "Missing system ID parameter '%s'", SYSTEM_ID_PARAMETER);
    return false;
  }

  if (systemId.empty())
  {
    RCLCPP_ERROR(m_node.get_logger(), "System ID parameter '%s' is empty", SYSTEM_ID_PARAMETER);
    return false;
  }

  const std::string imageTopic = systemId + "_" + IMAGE_TOPIC;
  const std::string flowTopic = systemId + "_" + FLOW_TOPIC;

  RCLCPP_INFO(m_node.get_logger(), "System ID: %s", systemId.c_str());
  RCLCPP_INFO(m_node.get_logger(), "Image topic: %s", imageTopic.c_str());
  RCLCPP_INFO(m_node.get_logger(), "Flow topic: %s", flowTopic.c_str());

  *m_flowPublisher = image_transport::create_publisher(&m_node, flowTopic);

  *m_imgSubscriber = image_transport::create_subscription(
      &m_node, imageTopic,
      [this](const sensor_msgs::msg::Image::ConstSharedPtr& msg) { OnImage(msg); }, "compressed");

  RCLCPP_INFO(m_node.get_logger(), "Started optical flow");

  return true;
}

void OpticalFlowNode::Deinitialize()
{
  m_imgSubscriber->shutdown();

  m_flowPublisher->shutdown();

  m_opticalFlow->Deinitialize();

  m_isInitialized = false;
  m_imageWidth = 0;
  m_imageHeight = 0;
}

void OpticalFlowNode::OnImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(m_node.get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  const cv::Mat& image = cv_ptr->image;
  if (image.empty())
  {
    RCLCPP_WARN(m_node.get_logger(), "Received empty image frame");
    return;
  }

  const int width = image.cols;
  const int height = image.rows;

  if (!m_isInitialized || width != m_imageWidth || height != m_imageHeight)
  {
    if (!m_opticalFlow->Initialize(m_logger, width, height))
    {
      RCLCPP_ERROR(m_node.get_logger(), "Failed to initialize optical flow for image size %dx%d",
                   width, height);
      return;
    }

    m_isInitialized = true;
    m_imageWidth = width;
    m_imageHeight = height;
  }

  if (!m_opticalFlow->ProcessImage(image))
  {
    RCLCPP_WARN(m_node.get_logger(), "Optical flow processing failed");
    return;
  }

  const auto points = m_opticalFlow->GetPoints();
  RCLCPP_INFO(m_node.get_logger(), "Tracked %zu optical flow points", points.size() / 2);

  for (size_t i = 0; i + 1 < points.size(); i += 2)
  {
    const cv::Point2f point(points[i], points[i + 1]);
    cv::circle(cv_ptr->image, point, 3, cv::Scalar(0, 0, 255), cv::FILLED);
  }

  m_flowPublisher->publish(cv_ptr->toImageMsg());
}
