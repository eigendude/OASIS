/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "PoseLandmarkRendererNode.h"

#include <algorithm>
#include <cstddef>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>

using namespace oasis_perception;

namespace
{
constexpr const char* IMAGE_TRANSPORT_PARAM = "image_transport";
constexpr const char* IMAGE_TRANSPORT_DEFAULT = "raw";
constexpr const char* IMAGE_TOPIC = "image";
constexpr const char* POSE_LANDMARKS_TOPIC = "pose_landmarks";
constexpr const char* CAMERA_SCENE_TOPIC = "camera_scene";
constexpr const char* POSE_IMAGE_TOPIC = "pose_image";

// Exact-time sync queue depth, matching the Python node's small frame map
constexpr int SYNC_QUEUE_SIZE = 4;

std::vector<PoseRenderInput> ToRenderInputs(const oasis_msgs::msg::PoseLandmarksArray& poseMsg,
                                            const oasis_msgs::msg::CameraScene& sceneMsg)
{
  std::vector<PoseRenderInput> inputs;
  inputs.reserve(poseMsg.poses.size());

  for (std::size_t i = 0; i < poseMsg.poses.size(); ++i)
  {
    PoseRenderInput input;
    input.landmarks.reserve(poseMsg.poses[i].landmarks.size());

    for (const auto& landmarkMsg : poseMsg.poses[i].landmarks)
    {
      input.landmarks.push_back({
          landmarkMsg.x,
          landmarkMsg.y,
          landmarkMsg.z,
      });
    }

    if (i < sceneMsg.bounding_boxes.size())
    {
      const auto& boxMsg = sceneMsg.bounding_boxes[i];
      input.boundingBox = {
          boxMsg.x_center,
          boxMsg.y_center,
          boxMsg.width,
          boxMsg.height,
      };
      input.hasBoundingBox = true;
    }

    inputs.emplace_back(std::move(input));
  }

  return inputs;
}
} // namespace

PoseLandmarkRendererNode::PoseLandmarkRendererNode(rclcpp::Node& node) : node(node)
{
}

PoseLandmarkRendererNode::~PoseLandmarkRendererNode() = default;

bool PoseLandmarkRendererNode::Start()
{
  imageTransport =
      node.declare_parameter<std::string>(IMAGE_TRANSPORT_PARAM, IMAGE_TRANSPORT_DEFAULT);

  if (imageTransport != IMAGE_TRANSPORT_DEFAULT)
  {
    RCLCPP_WARN(node.get_logger(),
                "Only raw image transport is supported by the exact-time renderer; "
                "got '%s'",
                imageTransport.c_str());
  }

  poseImagePub = image_transport::create_publisher(image_transport::RequiredInterfaces(node),
                                                   POSE_IMAGE_TOPIC, rclcpp::QoS(SYNC_QUEUE_SIZE));

  imageSub.subscribe(&node, IMAGE_TOPIC, rclcpp::SensorDataQoS());
  poseSub.subscribe(&node, POSE_LANDMARKS_TOPIC, rclcpp::QoS(SYNC_QUEUE_SIZE));
  sceneSub.subscribe(&node, CAMERA_SCENE_TOPIC, rclcpp::QoS(SYNC_QUEUE_SIZE));

  sync = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(SYNC_QUEUE_SIZE),
                                                                     imageSub, poseSub, sceneSub);

  sync->registerCallback(
      [this](const ImageMsg::ConstSharedPtr& imageMsg, const PoseMsg::ConstSharedPtr& poseMsg,
             const SceneMsg::ConstSharedPtr& sceneMsg)
      { RenderCallback(imageMsg, poseMsg, sceneMsg); });

  RCLCPP_INFO(node.get_logger(), "Pose landmark renderer initialized");

  return true;
}

void PoseLandmarkRendererNode::Stop()
{
  sync.reset();
  imageSub.unsubscribe();
  poseSub.unsubscribe();
  sceneSub.unsubscribe();
  poseImagePub.shutdown();
}

void PoseLandmarkRendererNode::RenderCallback(const ImageMsg::ConstSharedPtr& imageMsg,
                                              const PoseMsg::ConstSharedPtr& poseMsg,
                                              const SceneMsg::ConstSharedPtr& sceneMsg)
{
  cv_bridge::CvImagePtr imagePtr;
  try
  {
    imagePtr = cv_bridge::toCvCopy(imageMsg, sensor_msgs::image_encodings::BGR8);
  }
  catch (const cv_bridge::Exception& err)
  {
    RCLCPP_ERROR(node.get_logger(), "cv_bridge exception: %s", err.what());
    return;
  }

  std::vector<PoseRenderInput> renderInputs = ToRenderInputs(*poseMsg, *sceneMsg);
  renderer.Render(imagePtr->image, renderInputs);

  sensor_msgs::msg::Image::SharedPtr outputMsg;
  try
  {
    outputMsg = imagePtr->toImageMsg();
  }
  catch (const cv_bridge::Exception& err)
  {
    RCLCPP_ERROR(node.get_logger(), "Failed to convert annotated image: %s", err.what());
    return;
  }

  outputMsg->header = imageMsg->header;
  poseImagePub.publish(*outputMsg);
}
