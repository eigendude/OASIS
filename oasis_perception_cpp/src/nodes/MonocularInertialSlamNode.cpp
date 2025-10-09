/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "MonocularInertialSlamNode.h"

#include "slam/MonocularInertialSlam.h"

#include <functional>
#include <memory>
#include <string>

#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>

using namespace OASIS;
using namespace ROS;

namespace
{
// Subscribed topics
constexpr const char* IMAGE_TOPIC = "image";
constexpr const char* IMU_TOPIC = "imu";

// Parameters
constexpr const char* SYSTEM_ID_PARAMETER = "system_id";
constexpr const char* DEFAULT_SYSTEM_ID = "";
} // namespace

MonocularInertialSlamNode::MonocularInertialSlamNode(rclcpp::Node& node)
  : m_node(node),
    m_logger(std::make_unique<rclcpp::Logger>(node.get_logger())),
    m_imgSubscriber(std::make_unique<image_transport::Subscriber>())
{
  m_node.declare_parameter<std::string>(SYSTEM_ID_PARAMETER, DEFAULT_SYSTEM_ID);
}

MonocularInertialSlamNode::~MonocularInertialSlamNode() = default;

bool MonocularInertialSlamNode::Initialize()
{
  std::string systemId;
  if (!m_node.get_parameter(SYSTEM_ID_PARAMETER, systemId))
  {
    RCLCPP_ERROR(*m_logger, "Missing system ID parameter '%s'", SYSTEM_ID_PARAMETER);
    return false;
  }

  if (systemId.empty())
  {
    RCLCPP_ERROR(*m_logger, "System ID parameter '%s' is empty", SYSTEM_ID_PARAMETER);
    return false;
  }

  const std::string imageTopic = systemId + "_" + IMAGE_TOPIC;
  const std::string imuTopic = systemId + "_" + IMU_TOPIC;

  RCLCPP_INFO(*m_logger, "System ID: %s", systemId.c_str());
  RCLCPP_INFO(*m_logger, "Image topic: %s", imageTopic.c_str());
  RCLCPP_INFO(*m_logger, "IMU topic: %s", imuTopic.c_str());

  *m_imgSubscriber = image_transport::create_subscription(
      &m_node, imageTopic,
      [this](const sensor_msgs::msg::Image::ConstSharedPtr& msg) { OnImage(msg); }, "compressed");

  const rclcpp::QoS qos{10};
  m_imuSubscriber = m_node.create_subscription<oasis_msgs::msg::I2CImu>(
      imuTopic, qos, std::bind(&MonocularInertialSlamNode::OnImu, this, std::placeholders::_1));

  m_monocularInertialSlam = std::make_unique<SLAM::MonocularInertialSlam>(m_node);
  if (!m_monocularInertialSlam->Initialize())
  {
    RCLCPP_ERROR(*m_logger, "Failed to initialize monocular inertial SLAM");
    return false;
  }

  RCLCPP_INFO(*m_logger, "Started monocular inertial SLAM");

  return true;
}

void MonocularInertialSlamNode::Deinitialize()
{
  if (m_imgSubscriber)
    m_imgSubscriber->shutdown();

  m_imuSubscriber.reset();

  if (m_monocularInertialSlam)
  {
    m_monocularInertialSlam->Deinitialize();
    m_monocularInertialSlam.reset();
  }
}

void MonocularInertialSlamNode::OnImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  if (m_monocularInertialSlam)
    m_monocularInertialSlam->ReceiveImage(msg);
}

void MonocularInertialSlamNode::OnImu(const oasis_msgs::msg::I2CImu::ConstSharedPtr& msg)
{
  if (m_monocularInertialSlam)
    m_monocularInertialSlam->ImuCallback(msg);
}
