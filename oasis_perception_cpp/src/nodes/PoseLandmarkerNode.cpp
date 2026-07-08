/*
 *  Copyright (C) 2025-2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "PoseLandmarkerNode.h"

#include "pose/PoseLandmarker.h"

#include <algorithm>
#include <cinttypes>
#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_path.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <oasis_msgs/msg/bounding_box.hpp>
#include <oasis_msgs/msg/camera_scene.hpp>
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
constexpr const char* CAMERA_SCENE_TOPIC = "camera_scene";
constexpr const char* POSE_IMAGE_TOPIC = "pose_image";
constexpr const char* POSE_LANDMARKS_TOPIC = "pose_landmarks";
constexpr const char* IMAGE_TRANSPORT_PARAMETER = "image_transport";
constexpr const char* MODEL_ASSET_PATH_PARAMETER = "model_asset_path";
constexpr const char* MAX_POSES_PARAMETER = "max_poses";
constexpr const char* MIN_POSE_DETECTION_CONFIDENCE_PARAMETER = "min_pose_detection_confidence";
constexpr const char* MIN_POSE_PRESENCE_CONFIDENCE_PARAMETER = "min_pose_presence_confidence";
constexpr const char* MIN_TRACKING_CONFIDENCE_PARAMETER = "min_tracking_confidence";
constexpr const char* OUTPUT_SEGMENTATION_MASKS_PARAMETER = "output_segmentation_masks";
constexpr const char* LIVE_STREAM_TIMEOUT_MS_PARAMETER = "live_stream_timeout_ms";
constexpr const char* PUBLISH_POSE_IMAGE_PARAMETER = "publish_pose_image";
constexpr const char* DEFAULT_IMAGE_TRANSPORT = "raw";
constexpr const char* MODEL_PACKAGE_NAME = "oasis_perception_py";
constexpr const char* MODEL_RELATIVE_PATH = "mediapipe/pose_landmarker.task";
constexpr int DEFAULT_MAX_POSES = 5;
constexpr int MAX_POSES_BOUND = 5;
constexpr double DEFAULT_MIN_POSE_DETECTION_CONFIDENCE = 0.75;
constexpr double DEFAULT_MIN_POSE_PRESENCE_CONFIDENCE = 0.4;
constexpr double DEFAULT_MIN_TRACKING_CONFIDENCE = 0.4;
constexpr int DEFAULT_LIVE_STREAM_TIMEOUT_MS = 1000;

std::int64_t GetTimestampMs(const sensor_msgs::msg::Image& imageMsg)
{
  return static_cast<std::int64_t>(imageMsg.header.stamp.sec) * 1000 +
         static_cast<std::int64_t>(imageMsg.header.stamp.nanosec / 1000000);
}

std::string GetDefaultModelPath()
{
  return (ament_index_cpp::get_package_share_path(MODEL_PACKAGE_NAME) / MODEL_RELATIVE_PATH)
      .string();
}
} // namespace

PoseLandmarkerNode::PoseLandmarkerNode(rclcpp::Node& node)
  : m_node(node), m_poseLandmarker(std::make_unique<PoseLandmarker>())
{
  m_node.declare_parameter<std::string>(IMAGE_TRANSPORT_PARAMETER, DEFAULT_IMAGE_TRANSPORT);
  m_node.declare_parameter<std::string>(MODEL_ASSET_PATH_PARAMETER, "");
  m_node.declare_parameter<int>(MAX_POSES_PARAMETER, DEFAULT_MAX_POSES);
  m_node.declare_parameter<double>(MIN_POSE_DETECTION_CONFIDENCE_PARAMETER,
                                   DEFAULT_MIN_POSE_DETECTION_CONFIDENCE);
  m_node.declare_parameter<double>(MIN_POSE_PRESENCE_CONFIDENCE_PARAMETER,
                                   DEFAULT_MIN_POSE_PRESENCE_CONFIDENCE);
  m_node.declare_parameter<double>(MIN_TRACKING_CONFIDENCE_PARAMETER,
                                   DEFAULT_MIN_TRACKING_CONFIDENCE);
  m_node.declare_parameter<bool>(OUTPUT_SEGMENTATION_MASKS_PARAMETER, false);
  m_node.declare_parameter<int>(LIVE_STREAM_TIMEOUT_MS_PARAMETER, DEFAULT_LIVE_STREAM_TIMEOUT_MS);
  m_node.declare_parameter<bool>(PUBLISH_POSE_IMAGE_PARAMETER, false);
}

PoseLandmarkerNode::~PoseLandmarkerNode() = default;

bool PoseLandmarkerNode::Start()
{
  std::string modelPath;
  if (!m_node.get_parameter(MODEL_ASSET_PATH_PARAMETER, modelPath) || modelPath.empty())
    modelPath = GetDefaultModelPath();

  const auto maxPosesValue = std::clamp<std::int64_t>(
      m_node.get_parameter(MAX_POSES_PARAMETER).as_int(), 1, MAX_POSES_BOUND);
  const int maxPoses = static_cast<int>(maxPosesValue);
  const double minPoseDetectionConfidence = std::clamp(
      m_node.get_parameter(MIN_POSE_DETECTION_CONFIDENCE_PARAMETER).as_double(), 0.0, 1.0);
  const double minPosePresenceConfidence = std::clamp(
      m_node.get_parameter(MIN_POSE_PRESENCE_CONFIDENCE_PARAMETER).as_double(), 0.0, 1.0);
  const double minTrackingConfidence =
      std::clamp(m_node.get_parameter(MIN_TRACKING_CONFIDENCE_PARAMETER).as_double(), 0.0, 1.0);
  const auto liveStreamTimeoutMsValue =
      std::max<std::int64_t>(m_node.get_parameter(LIVE_STREAM_TIMEOUT_MS_PARAMETER).as_int(), 1);
  const int liveStreamTimeoutMs = static_cast<int>(liveStreamTimeoutMsValue);
  m_publishPoseImage = m_node.get_parameter(PUBLISH_POSE_IMAGE_PARAMETER).as_bool();

  const mediapipe_facade::PoseLandmarkerConfig config{
      .loggingName = m_node.get_name(),
      .modelAssetPath = modelPath,
      .maxPoses = maxPoses,
      .minPoseDetectionConfidence = static_cast<float>(minPoseDetectionConfidence),
      .minPosePresenceConfidence = static_cast<float>(minPosePresenceConfidence),
      .minTrackingConfidence = static_cast<float>(minTrackingConfidence),
      .outputSegmentationMasks =
          m_node.get_parameter(OUTPUT_SEGMENTATION_MASKS_PARAMETER).as_bool(),
      .liveStreamTimeoutMs = liveStreamTimeoutMs,
  };

  if (!m_poseLandmarker->Initialize(config))
  {
    RCLCPP_ERROR(m_node.get_logger(), "PoseLandmarker initialization failed");
    return false;
  }

  RCLCPP_INFO(m_node.get_logger(), "PoseLandmarker initialized: %s",
              m_poseLandmarker->LastStatusMessage().c_str());

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
  m_lastSubmittedTimestampMs = 0;
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
  if (m_publishPoseImage)
    m_publisher = image_transport::create_publisher(&m_node, POSE_IMAGE_TOPIC);
  m_landmarksPublisher =
      m_node.create_publisher<oasis_msgs::msg::PoseLandmarksArray>(POSE_LANDMARKS_TOPIC, 10);
  m_cameraScenePublisher =
      m_node.create_publisher<oasis_msgs::msg::CameraScene>(CAMERA_SCENE_TOPIC, 10);

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
  std::int64_t timestampMs = GetTimestampMs(imageMsg);
  if (timestampMs <= m_lastSubmittedTimestampMs)
  {
    timestampMs = m_lastSubmittedTimestampMs + 1;
    ++m_adjustedTimestampCount;
    if (m_adjustedTimestampCount <= 5 || m_adjustedTimestampCount % 100 == 0)
    {
      RCLCPP_DEBUG(m_node.get_logger(),
                   "Adjusted pose live-stream timestamp to %" PRId64 " ms after %" PRId64
                   " adjustments",
                   timestampMs, m_adjustedTimestampCount);
    }
  }
  m_lastSubmittedTimestampMs = timestampMs;

  const mediapipe_facade::PoseDetectionInput input{
      .width = imagePtr->image.cols,
      .height = imagePtr->image.rows,
      .channelCount = imagePtr->image.channels(),
      .encoding = sensor_msgs::image_encodings::RGB8,
      .data = rgbImage.data,
      .dataSize = dataSize,
      .timestampMs = timestampMs,
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
  oasis_msgs::msg::CameraScene cameraSceneMsg;
  landmarksMsg.header = imageMsg.header;
  cameraSceneMsg.header = imageMsg.header;

  for (const std::vector<mediapipe_facade::PoseLandmark>& pose : result.poses)
  {
    oasis_msgs::msg::PoseLandmarks poseMsg;
    float minX = 1.0F;
    float minY = 1.0F;
    float maxX = 0.0F;
    float maxY = 0.0F;

    for (const mediapipe_facade::PoseLandmark& landmark : pose)
    {
      oasis_msgs::msg::Landmark landmarkMsg;
      landmarkMsg.x = landmark.x;
      landmarkMsg.y = landmark.y;
      landmarkMsg.z = landmark.z;
      poseMsg.landmarks.push_back(landmarkMsg);

      minX = std::min(minX, landmark.x);
      minY = std::min(minY, landmark.y);
      maxX = std::max(maxX, landmark.x);
      maxY = std::max(maxY, landmark.y);
    }

    if (!pose.empty())
    {
      oasis_msgs::msg::BoundingBox bboxMsg;
      bboxMsg.x_center = (minX + maxX) * 0.5F;
      bboxMsg.y_center = (minY + maxY) * 0.5F;
      bboxMsg.width = maxX - minX;
      bboxMsg.height = maxY - minY;
      cameraSceneMsg.bounding_boxes.push_back(bboxMsg);
    }

    landmarksMsg.poses.push_back(std::move(poseMsg));
  }
  m_landmarksPublisher->publish(landmarksMsg);
  m_cameraScenePublisher->publish(cameraSceneMsg);

  if (m_publishPoseImage && m_publisher.getNumSubscribers() > 0)
  {
    sensor_msgs::msg::Image::SharedPtr outMsg = imagePtr->toImageMsg();
    m_publisher.publish(*outMsg);
  }
}
