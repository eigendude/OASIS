/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */
#pragma once

#include "pose/PoseLandmarkRenderer.h"

#include <memory>
#include <string>

#include <image_transport/publisher.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/exact_time.hpp>
#include <message_filters/synchronizer.hpp>
#include <oasis_msgs/msg/camera_scene.hpp>
#include <oasis_msgs/msg/pose_landmarks_array.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace rclcpp
{
class Node;
}

namespace oasis_perception
{
class PoseLandmarkRendererNode
{
public:
  explicit PoseLandmarkRendererNode(rclcpp::Node& node);
  ~PoseLandmarkRendererNode();

  bool Start();
  void Stop();

private:
  using ImageMsg = sensor_msgs::msg::Image;
  using PoseMsg = oasis_msgs::msg::PoseLandmarksArray;
  using SceneMsg = oasis_msgs::msg::CameraScene;
  using SyncPolicy = message_filters::sync_policies::ExactTime<ImageMsg, PoseMsg, SceneMsg>;

  void RenderCallback(const ImageMsg::ConstSharedPtr& imageMsg,
                      const PoseMsg::ConstSharedPtr& poseMsg,
                      const SceneMsg::ConstSharedPtr& sceneMsg);

  rclcpp::Node& node;

  std::string imageTransport;
  image_transport::Publisher poseImagePub;

  message_filters::Subscriber<ImageMsg> imageSub;
  message_filters::Subscriber<PoseMsg> poseSub;
  message_filters::Subscriber<SceneMsg> sceneSub;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync;

  PoseLandmarkRenderer renderer;
};
} // namespace oasis_perception
