/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "OpticalFlowNode.h"

#include "video/OpticalFlow.h"

#include <algorithm>
#include <functional>
#include <string>
#include <string_view>

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
constexpr std::string_view FLOW_TOPIC = "flow";

// Optical flow configuration
constexpr unsigned int MAX_TRACKED_POINTS = 60;

// Subscribed topics
constexpr std::string_view IMAGE_TOPIC = "image";

// Parameters
constexpr std::string_view SYSTEM_ID_PARAMETER = "system_id";
constexpr std::string_view DEFAULT_SYSTEM_ID = "";
constexpr std::string_view IMAGE_TRANSPORT_PARAMETER = "image_transport";
constexpr std::string_view DEFAULT_IMAGE_TRANSPORT = "raw";
} // namespace

OpticalFlowNode::OpticalFlowNode(rclcpp::Node& node)
  : m_node(node),
    m_logger(node.get_logger()),
    m_flowPublisher(std::make_unique<image_transport::Publisher>()),
    m_imgSubscriber(std::make_unique<image_transport::Subscriber>()),
    m_opticalFlow(std::make_unique<VIDEO::OpticalFlow>())
{
  m_node.declare_parameter<std::string>(SYSTEM_ID_PARAMETER.data(), DEFAULT_SYSTEM_ID.data());
  m_node.declare_parameter<std::string>(IMAGE_TRANSPORT_PARAMETER.data(),
                                        DEFAULT_IMAGE_TRANSPORT.data());

  VIDEO::ConfigOptions configOptions;
  configOptions.maxPointCount = MAX_TRACKED_POINTS;
  m_opticalFlow->SetConfig(configOptions);
}

OpticalFlowNode::~OpticalFlowNode() = default;

bool OpticalFlowNode::Initialize()
{
  RCLCPP_INFO(m_node.get_logger(), "Starting optical flow...");

  std::string systemId;
  if (!m_node.get_parameter(SYSTEM_ID_PARAMETER.data(), systemId))
  {
    RCLCPP_ERROR(m_node.get_logger(), "Missing system ID parameter '%s'",
                 SYSTEM_ID_PARAMETER.data());
    return false;
  }

  if (systemId.empty())
  {
    RCLCPP_ERROR(m_node.get_logger(), "System ID parameter '%s' is empty",
                 SYSTEM_ID_PARAMETER.data());
    return false;
  }

  std::string imageTransport;
  if (!m_node.get_parameter(IMAGE_TRANSPORT_PARAMETER.data(), imageTransport))
  {
    RCLCPP_ERROR(m_node.get_logger(), "Missing image transport parameter '%s'",
                 IMAGE_TRANSPORT_PARAMETER.data());
    return false;
  }

  if (imageTransport.empty())
    imageTransport = std::string{DEFAULT_IMAGE_TRANSPORT};

  std::string imageTopic = systemId;
  imageTopic.push_back('_');
  imageTopic.append(IMAGE_TOPIC);

  std::string flowTopic = systemId;
  flowTopic.push_back('_');
  flowTopic.append(FLOW_TOPIC);

  RCLCPP_INFO(m_node.get_logger(), "System ID: %s", systemId.c_str());
  RCLCPP_INFO(m_node.get_logger(), "Image topic: %s", imageTopic.c_str());
  RCLCPP_INFO(m_node.get_logger(), "Image transport: %s", imageTransport.c_str());
  RCLCPP_INFO(m_node.get_logger(), "Flow topic: %s", flowTopic.c_str());

  *m_flowPublisher = image_transport::create_publisher(&m_node, flowTopic);

  *m_imgSubscriber = image_transport::create_subscription(
      &m_node, imageTopic,
      [this](const sensor_msgs::msg::Image::ConstSharedPtr& msg) { OnImage(msg); }, imageTransport);

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

  const size_t trackedPointCount = m_opticalFlow->DrawPoints(cv_ptr->image, MAX_TRACKED_POINTS);
  //RCLCPP_INFO(m_node.get_logger(), "Tracked %zu optical flow points", trackedPointCount);

  m_flowPublisher->publish(cv_ptr->toImageMsg());
}
