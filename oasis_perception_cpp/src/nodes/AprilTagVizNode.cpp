/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "nodes/AprilTagVizNode.h"

#include <string>

#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>

using apriltag_msgs::msg::AprilTagDetectionArray;
using sensor_msgs::msg::Image;

namespace
{
// Topic suffixes
constexpr std::string_view IMAGE_TOPIC = "image";
constexpr std::string_view DETECTIONS_TOPIC = "apriltags";
constexpr std::string_view OVERLAY_TOPIC = "apriltags_image";

// Parameters
constexpr std::string_view SYSTEM_ID_PARAMETER = "system_id";
constexpr std::string_view IMAGE_TRANSPORT_PARAMETER = "image_transport";

constexpr std::string_view DEFAULT_SYSTEM_ID = "";
constexpr std::string_view DEFAULT_IMAGE_TRANSPORT = "raw";
} // namespace

using namespace OASIS;

AprilTagVizNode::AprilTagVizNode(rclcpp::Node& node)
  : m_node(node),
    m_logger(node.get_logger()),
    m_visualizer(node.get_logger(), node.get_clock()),
    m_overlayPublisher(std::make_unique<image_transport::Publisher>()),
    m_imageSubscription(std::make_unique<image_transport::Subscriber>())
{
  m_node.declare_parameter<std::string>(SYSTEM_ID_PARAMETER.data(), DEFAULT_SYSTEM_ID.data());
  m_node.declare_parameter<std::string>(IMAGE_TRANSPORT_PARAMETER.data(),
                                        DEFAULT_IMAGE_TRANSPORT.data());
}

AprilTagVizNode::~AprilTagVizNode() = default;

bool AprilTagVizNode::Initialize()
{
  if (!m_node.get_parameter(SYSTEM_ID_PARAMETER.data(), m_systemId) || m_systemId.empty())
  {
    RCLCPP_ERROR(m_logger, "Missing or empty system ID parameter '%s'", SYSTEM_ID_PARAMETER.data());
    return false;
  }

  m_node.get_parameter(IMAGE_TRANSPORT_PARAMETER.data(), m_imageTransport);

  std::string imageTopic = m_systemId;
  imageTopic.push_back('_');
  imageTopic.append(IMAGE_TOPIC);

  std::string detectionsTopic = m_systemId;
  detectionsTopic.push_back('_');
  detectionsTopic.append(DETECTIONS_TOPIC);

  std::string overlayTopic = m_systemId;
  overlayTopic.push_back('_');
  overlayTopic.append(OVERLAY_TOPIC);

  RCLCPP_INFO(m_logger, "System ID: %s", m_systemId.c_str());
  RCLCPP_INFO(m_logger, "Image topic: %s", imageTopic.c_str());
  RCLCPP_INFO(m_logger, "Detections topic: %s", detectionsTopic.c_str());
  RCLCPP_INFO(m_logger, "Overlay topic: %s", overlayTopic.c_str());
  RCLCPP_INFO(m_logger, "Image transport: %s", m_imageTransport.c_str());

  // Publishers
  *m_overlayPublisher = image_transport::create_publisher(&m_node, overlayTopic);

  // Subscribers
  *m_imageSubscription = image_transport::create_subscription(
      &m_node, imageTopic, [this](const Image::ConstSharedPtr& msg)
      { AprilTagVizNode::OnImage(msg); }, m_imageTransport, rclcpp::QoS{1}.get_rmw_qos_profile());
  m_detectionSubscription = m_node.create_subscription<AprilTagDetectionArray>(
      detectionsTopic, rclcpp::QoS{1}, [this](const AprilTagDetectionArray::ConstSharedPtr& msg)
      { AprilTagVizNode::OnDetections(msg); });

  return true;
}

void AprilTagVizNode::Deinitialize()
{
  m_detectionSubscription.reset();
  m_imageSubscription->shutdown();
  m_overlayPublisher->shutdown();
}

void AprilTagVizNode::OnImage(const Image::ConstSharedPtr& msg)
{
  if (!m_overlayPublisher)
    return;

  sensor_msgs::msg::Image::SharedPtr output = m_visualizer.ProcessImage(msg);
  if (output == nullptr)
    return;

  m_overlayPublisher->publish(output);
}

void AprilTagVizNode::OnDetections(const AprilTagDetectionArray::ConstSharedPtr& msg)
{
  m_visualizer.ProcessDetections(msg);
}
