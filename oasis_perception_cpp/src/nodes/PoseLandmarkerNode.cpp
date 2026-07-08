/*
 *  Copyright (C) 2025-2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "PoseLandmarkerNode.h"

#include "pose/PoseLandmarker.h"

#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_path.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <oasis_msgs/msg/landmark.hpp>
#include <oasis_msgs/msg/pose_landmarks.hpp>
#include <oasis_msgs/msg/pose_landmarks_array.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace oasis_perception;

namespace
{
constexpr const char* IMAGE_TOPIC = "image";
constexpr const char* POSE_IMAGE_TOPIC = "pose_image";
constexpr const char* POSE_LANDMARKS_TOPIC = "pose_landmarks";
constexpr const char* IMAGE_TRANSPORT_PARAMETER = "image_transport";
constexpr const char* DEFAULT_IMAGE_TRANSPORT = "raw";
constexpr const char* MODEL_PACKAGE_NAME = "oasis_perception_py";
constexpr const char* MODEL_RELATIVE_PATH = "mediapipe/pose_landmarker.task";
constexpr int MAX_POSES = 5;

std::int64_t GetTimestampMs(const sensor_msgs::msg::Image& imageMsg)
{
  return static_cast<std::int64_t>(imageMsg.header.stamp.sec) * 1000 +
         static_cast<std::int64_t>(imageMsg.header.stamp.nanosec / 1000000);
}
} // namespace

PoseLandmarkerNode::PoseLandmarkerNode(rclcpp::Node& node)
  : m_node(node), m_poseLandmarker(std::make_unique<PoseLandmarker>())
{
  m_node.declare_parameter<std::string>(IMAGE_TRANSPORT_PARAMETER, DEFAULT_IMAGE_TRANSPORT);
}

PoseLandmarkerNode::~PoseLandmarkerNode() = default;

bool PoseLandmarkerNode::Start()
{
  const std::string modelPath =
      (ament_index_cpp::get_package_share_path(MODEL_PACKAGE_NAME) / MODEL_RELATIVE_PATH).string();
  const mediapipe_facade::PoseLandmarkerConfig config{
      .loggingName = m_node.get_name(),
      .modelAssetPath = modelPath,
      .maxPoses = MAX_POSES,
  };

  if (!m_poseLandmarker->Initialize(config))
  {
    RCLCPP_ERROR(m_node.get_logger(), "PoseLandmarker initialization failed");
    return false;
  }

  RCLCPP_INFO(m_node.get_logger(), "PoseLandmarker initialized");

  const int startupWidth = 256;
  const int startupHeight = 256;
  const int startupChannels = 3;
  const std::vector<std::uint8_t> startupImage(
      static_cast<std::size_t>(startupWidth * startupHeight * startupChannels), 0);
  const mediapipe_facade::PoseDetectionInput startupInput{
      .width = startupWidth,
      .height = startupHeight,
      .channelCount = startupChannels,
      .encoding = "startup",
      .data = startupImage.data(),
      .dataSize = startupImage.size(),
      .timestampMs = 0,
  };
  const mediapipe_facade::PoseDetectionResult startupResult =
      m_poseLandmarker->Detect(startupInput);
  if (!startupResult.success)
  {
    RCLCPP_ERROR(m_node.get_logger(), "PoseLandmarker facade startup check failed: %s",
                 startupResult.message.c_str());
    return false;
  }

  RCLCPP_INFO(m_node.get_logger(),
              "PoseLandmarker facade startup check succeeded with %zu poses: %s",
              startupResult.poses.size(), startupResult.message.c_str());

  std::string imageTransport;
  if (!m_node.get_parameter(IMAGE_TRANSPORT_PARAMETER, imageTransport) || imageTransport.empty())
    imageTransport = DEFAULT_IMAGE_TRANSPORT;

  m_subscriber = image_transport::create_subscription(
      &m_node, IMAGE_TOPIC,
      [this](const sensor_msgs::msg::Image::ConstSharedPtr& imageMsg)
      {
        if (imageMsg)
          OnImage(*imageMsg);
      },
      imageTransport);
  m_publisher = image_transport::create_publisher(&m_node, POSE_IMAGE_TOPIC);
  m_landmarksPublisher =
      m_node.create_publisher<oasis_msgs::msg::PoseLandmarksArray>(POSE_LANDMARKS_TOPIC, 10);

  return true;
}

void PoseLandmarkerNode::Stop()
{
  m_subscriber.shutdown();
}

void PoseLandmarkerNode::OnImage(const sensor_msgs::msg::Image& imageMsg)
{
  // Convert the incoming image to an RGB image for MediaPipe
  cv_bridge::CvImageConstPtr imagePtr;
  try
  {
    imagePtr = cv_bridge::toCvCopy(imageMsg, sensor_msgs::image_encodings::RGB8);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(m_node.get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  const cv::Mat& rgbImage = imagePtr->image;
  if (!rgbImage.isContinuous())
  {
    RCLCPP_ERROR(m_node.get_logger(), "PoseLandmarker RGB image is not contiguous");
    return;
  }

  const std::size_t dataSize = rgbImage.total() * rgbImage.elemSize();
  const mediapipe_facade::PoseDetectionInput input{
      .width = imagePtr->image.cols,
      .height = imagePtr->image.rows,
      .channelCount = imagePtr->image.channels(),
      .encoding = sensor_msgs::image_encodings::RGB8,
      .data = rgbImage.data,
      .dataSize = dataSize,
      .timestampMs = GetTimestampMs(imageMsg),
  };
  const mediapipe_facade::PoseDetectionResult result = m_poseLandmarker->Detect(input);
  if (!result.success)
  {
    RCLCPP_ERROR(m_node.get_logger(), "PoseLandmarker facade detection failed: %s",
                 result.message.c_str());
    return;
  }

  RCLCPP_DEBUG(m_node.get_logger(), "PoseLandmarker detected %zu poses", result.poses.size());

  oasis_msgs::msg::PoseLandmarksArray landmarksMsg;
  landmarksMsg.header = imageMsg.header;
  for (const std::vector<mediapipe_facade::PoseLandmark>& pose : result.poses)
  {
    oasis_msgs::msg::PoseLandmarks poseMsg;
    for (const mediapipe_facade::PoseLandmark& landmark : pose)
    {
      oasis_msgs::msg::Landmark landmarkMsg;
      landmarkMsg.x = landmark.x;
      landmarkMsg.y = landmark.y;
      landmarkMsg.z = landmark.z;
      poseMsg.landmarks.push_back(landmarkMsg);
    }
    landmarksMsg.poses.push_back(std::move(poseMsg));
  }
  m_landmarksPublisher->publish(landmarksMsg);

  sensor_msgs::msg::Image::SharedPtr outMsg = imagePtr->toImageMsg();

  if (m_publisher.getNumSubscribers() > 0)
    m_publisher.publish(*outMsg);
}
