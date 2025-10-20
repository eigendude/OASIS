/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */
#pragma once

#include <memory>

#include <image_transport/publisher.hpp>
#include <image_transport/subscriber.hpp>
#include <oasis_msgs/msg/scene_score.hpp>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace rclcpp
{
class Node;
}

namespace OASIS
{
namespace VIDEO
{
class OpticalFlow;
}

namespace ROS
{
class OpticalFlowNode
{
public:
  OpticalFlowNode(rclcpp::Node& node);
  ~OpticalFlowNode();

  bool Initialize();
  void Deinitialize();

private:
  // ROS interface
  void OnImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

  // Construction parameters
  rclcpp::Node& m_node;

  // ROS parameters
  rclcpp::Logger m_logger;
  std::unique_ptr<image_transport::Publisher> m_flowPublisher;
  std::unique_ptr<image_transport::Subscriber> m_imgSubscriber;
  rclcpp::Publisher<oasis_msgs::msg::SceneScore>::SharedPtr m_scenePublisher;

  // Video parameters
  std::unique_ptr<VIDEO::OpticalFlow> m_opticalFlow;

  // State parameters
  bool m_publishSceneScores{false};
  bool m_isInitialized{false};
  int m_imageWidth{0};
  int m_imageHeight{0};
};
} // namespace ROS
} // namespace OASIS
