/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "CheckerboardDetectorNode.h"

#include "calibration/CheckerboardDetector.h"

#include <exception>
#include <string>
#include <string_view>

#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace
{
constexpr std::string_view IMAGE_TOPIC = "image";
constexpr std::string_view STATUS_TOPIC = "checkerboard_status";
constexpr std::string_view DEBUG_IMAGE_TOPIC = "checkerboard_image";

// ROS parameter: "camera_model"
// Type: string
// Default: "pinhole"
// Meaning: Informational and reserved for future camera-model-specific
// behavior. The detector currently only uses checkerboard visibility.
constexpr std::string_view CAMERA_MODEL_PARAMETER = "camera_model";
constexpr std::string_view DEFAULT_CAMERA_MODEL = "pinhole";

// ROS parameter: "checkerboard_width"
// Type: integer
// Default: 8
// Meaning: Number of inner checkerboard corners along image x, not squares.
// Valid range: > 0 corners.
constexpr std::string_view CHECKERBOARD_WIDTH_PARAMETER = "checkerboard_width";
constexpr int64_t DEFAULT_CHECKERBOARD_WIDTH = 8;

// ROS parameter: "checkerboard_height"
// Type: integer
// Default: 6
// Meaning: Number of inner checkerboard corners along image y, not squares.
// Valid range: > 0 corners.
constexpr std::string_view CHECKERBOARD_HEIGHT_PARAMETER = "checkerboard_height";
constexpr int64_t DEFAULT_CHECKERBOARD_HEIGHT = 6;

// ROS parameter: "image_transport"
// Type: string
// Default: "raw"
// Meaning: Transport passed to image_transport::create_subscription. Use
// "compressed" for station SD compressed input.
constexpr std::string_view IMAGE_TRANSPORT_PARAMETER = "image_transport";
constexpr std::string_view DEFAULT_IMAGE_TRANSPORT = "raw";

// ROS parameter: "publish_debug_image"
// Type: bool
// Default: true
// Meaning: Publishes checkerboard_image with OpenCV corner overlays when true.
constexpr std::string_view PUBLISH_DEBUG_IMAGE_PARAMETER = "publish_debug_image";
constexpr bool DEFAULT_PUBLISH_DEBUG_IMAGE = true;

// ROS parameter: "use_sb_detector"
// Type: bool
// Default: true
// Meaning: Uses cv::findChessboardCornersSB when true, otherwise classic
// cv::findChessboardCorners.
constexpr std::string_view USE_SB_DETECTOR_PARAMETER = "use_sb_detector";
constexpr bool DEFAULT_USE_SB_DETECTOR = true;

// ROS parameter: "refine_corners"
// Type: bool
// Default: true
// Meaning: Classic detector only. Applies cv::cornerSubPix after successful
// classic checkerboard detection.
constexpr std::string_view REFINE_CORNERS_PARAMETER = "refine_corners";
constexpr bool DEFAULT_REFINE_CORNERS = true;

// ROS parameter: "processing_interval"
// Type: integer
// Default: 1
// Meaning: Process every Nth received image frame.
// Valid range: > 0 frames.
constexpr std::string_view PROCESSING_INTERVAL_PARAMETER = "processing_interval";
constexpr int64_t DEFAULT_PROCESSING_INTERVAL = 1;

// ROS parameter: "adaptive_thresh"
// Type: bool
// Default: true
// Meaning: Classic detector only. Maps to cv::CALIB_CB_ADAPTIVE_THRESH.
constexpr std::string_view ADAPTIVE_THRESH_PARAMETER = "adaptive_thresh";
constexpr bool DEFAULT_ADAPTIVE_THRESH = true;

// ROS parameter: "normalize_image"
// Type: bool
// Default: true
// Meaning: Applies OpenCV image normalization checkerboard flag. Maps to
// cv::CALIB_CB_NORMALIZE_IMAGE for both SB and classic detector paths.
constexpr std::string_view NORMALIZE_IMAGE_PARAMETER = "normalize_image";
constexpr bool DEFAULT_NORMALIZE_IMAGE = true;

// ROS parameter: "fast_check"
// Type: bool
// Default: true
// Meaning: Classic detector only. Maps to cv::CALIB_CB_FAST_CHECK.
constexpr std::string_view FAST_CHECK_PARAMETER = "fast_check";
constexpr bool DEFAULT_FAST_CHECK = true;

bool IsMonoEncoding(const std::string& encoding)
{
  return encoding == sensor_msgs::image_encodings::MONO8 ||
         encoding == sensor_msgs::image_encodings::MONO16 ||
         encoding == sensor_msgs::image_encodings::TYPE_8UC1 ||
         encoding == sensor_msgs::image_encodings::TYPE_16UC1;
}

cv::Mat ConvertToGray(const cv_bridge::CvImageConstPtr& cvImagePtr)
{
  const cv::Mat& image = cvImagePtr->image;
  const std::string& encoding = cvImagePtr->encoding;

  if (IsMonoEncoding(encoding))
  {
    if (image.depth() == CV_8U)
      return image;

    cv::Mat gray8;
    image.convertTo(gray8, CV_8U, 255.0 / 65535.0);
    return gray8;
  }

  cv::Mat gray;
  if (encoding == sensor_msgs::image_encodings::BGR8)
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  else if (encoding == sensor_msgs::image_encodings::RGB8)
    cv::cvtColor(image, gray, cv::COLOR_RGB2GRAY);
  else if (encoding == sensor_msgs::image_encodings::BGRA8)
    cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
  else if (encoding == sensor_msgs::image_encodings::RGBA8)
    cv::cvtColor(image, gray, cv::COLOR_RGBA2GRAY);
  else if (image.channels() == 1)
    gray = image;
  else if (image.channels() == 3)
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  else if (image.channels() == 4)
    cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
  else
    throw std::invalid_argument("Unsupported image encoding: " + encoding);

  return gray;
}
} // namespace

namespace OASIS
{

CheckerboardDetectorNode::CheckerboardDetectorNode(rclcpp::Node& node)
  : m_node(node),
    m_debugPublisher(std::make_unique<image_transport::Publisher>()),
    m_imageSubscriber(std::make_unique<image_transport::Subscriber>())
{
  m_node.declare_parameter<std::string>(CAMERA_MODEL_PARAMETER.data(), DEFAULT_CAMERA_MODEL.data());
  m_node.declare_parameter<int64_t>(CHECKERBOARD_WIDTH_PARAMETER.data(),
                                    DEFAULT_CHECKERBOARD_WIDTH);
  m_node.declare_parameter<int64_t>(CHECKERBOARD_HEIGHT_PARAMETER.data(),
                                    DEFAULT_CHECKERBOARD_HEIGHT);
  m_node.declare_parameter<std::string>(IMAGE_TRANSPORT_PARAMETER.data(),
                                        DEFAULT_IMAGE_TRANSPORT.data());
  m_node.declare_parameter<bool>(PUBLISH_DEBUG_IMAGE_PARAMETER.data(), DEFAULT_PUBLISH_DEBUG_IMAGE);
  m_node.declare_parameter<bool>(USE_SB_DETECTOR_PARAMETER.data(), DEFAULT_USE_SB_DETECTOR);
  m_node.declare_parameter<bool>(REFINE_CORNERS_PARAMETER.data(), DEFAULT_REFINE_CORNERS);
  m_node.declare_parameter<int64_t>(PROCESSING_INTERVAL_PARAMETER.data(),
                                    DEFAULT_PROCESSING_INTERVAL);
  m_node.declare_parameter<bool>(ADAPTIVE_THRESH_PARAMETER.data(), DEFAULT_ADAPTIVE_THRESH);
  m_node.declare_parameter<bool>(NORMALIZE_IMAGE_PARAMETER.data(), DEFAULT_NORMALIZE_IMAGE);
  m_node.declare_parameter<bool>(FAST_CHECK_PARAMETER.data(), DEFAULT_FAST_CHECK);
}

CheckerboardDetectorNode::~CheckerboardDetectorNode() = default;

bool CheckerboardDetectorNode::Initialize()
{
  RCLCPP_INFO(m_node.get_logger(), "Starting checkerboard detector...");

  std::string cameraModel;
  m_node.get_parameter(CAMERA_MODEL_PARAMETER.data(), cameraModel);
  if (cameraModel.empty())
    cameraModel = std::string{DEFAULT_CAMERA_MODEL};

  std::string imageTransport;
  m_node.get_parameter(IMAGE_TRANSPORT_PARAMETER.data(), imageTransport);
  if (imageTransport.empty())
    imageTransport = std::string{DEFAULT_IMAGE_TRANSPORT};

  CALIBRATION::CheckerboardDetectorOptions options{};
  int64_t checkerboardWidth = DEFAULT_CHECKERBOARD_WIDTH;
  int64_t checkerboardHeight = DEFAULT_CHECKERBOARD_HEIGHT;
  m_node.get_parameter(CHECKERBOARD_WIDTH_PARAMETER.data(), checkerboardWidth);
  m_node.get_parameter(CHECKERBOARD_HEIGHT_PARAMETER.data(), checkerboardHeight);

  if (checkerboardWidth <= 0 || checkerboardHeight <= 0)
  {
    RCLCPP_ERROR(m_node.get_logger(), "Checkerboard dimensions must be positive");
    return false;
  }

  options.checkerboardWidth = static_cast<int>(checkerboardWidth);
  options.checkerboardHeight = static_cast<int>(checkerboardHeight);
  m_node.get_parameter(USE_SB_DETECTOR_PARAMETER.data(), options.useSbDetector);
  m_node.get_parameter(REFINE_CORNERS_PARAMETER.data(), options.refineCorners);
  m_node.get_parameter(ADAPTIVE_THRESH_PARAMETER.data(), options.adaptiveThresh);
  m_node.get_parameter(NORMALIZE_IMAGE_PARAMETER.data(), options.normalizeImage);
  m_node.get_parameter(FAST_CHECK_PARAMETER.data(), options.fastCheck);

  m_node.get_parameter(PUBLISH_DEBUG_IMAGE_PARAMETER.data(), m_publishDebugImage);
  m_node.get_parameter(PROCESSING_INTERVAL_PARAMETER.data(), m_processingInterval);
  if (m_processingInterval <= 0)
  {
    RCLCPP_WARN(m_node.get_logger(), "Invalid processing interval %ld. Using 1.",
                m_processingInterval);
    m_processingInterval = DEFAULT_PROCESSING_INTERVAL;
  }

  try
  {
    m_detector = std::make_unique<CALIBRATION::CheckerboardDetector>(options);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(m_node.get_logger(), "Failed to initialize checkerboard detector: %s", e.what());
    return false;
  }

  RCLCPP_INFO(m_node.get_logger(), "Camera model: %s", cameraModel.c_str());
  RCLCPP_INFO(m_node.get_logger(), "Image topic: %s", IMAGE_TOPIC.data());
  RCLCPP_INFO(m_node.get_logger(), "Image transport: %s", imageTransport.c_str());
  RCLCPP_INFO(m_node.get_logger(), "Status topic: %s", STATUS_TOPIC.data());
  RCLCPP_INFO(m_node.get_logger(), "Debug image topic: %s", DEBUG_IMAGE_TOPIC.data());
  RCLCPP_INFO(m_node.get_logger(), "Checkerboard inner corners: %dx%d", options.checkerboardWidth,
              options.checkerboardHeight);
  RCLCPP_INFO(m_node.get_logger(), "Using SB detector: %s",
              options.useSbDetector ? "true" : "false");
  RCLCPP_INFO(m_node.get_logger(), "Publishing debug image: %s",
              m_publishDebugImage ? "true" : "false");
  RCLCPP_INFO(m_node.get_logger(), "Processing interval: %ld", m_processingInterval);

  m_statusPublisher = m_node.create_publisher<std_msgs::msg::Bool>(
      STATUS_TOPIC.data(), rclcpp::SensorDataQoS().keep_last(1));

  if (m_publishDebugImage)
    *m_debugPublisher = image_transport::create_publisher(&m_node, DEBUG_IMAGE_TOPIC.data());

  *m_imageSubscriber = image_transport::create_subscription(
      &m_node, IMAGE_TOPIC.data(),
      [this](const sensor_msgs::msg::Image::ConstSharedPtr& imageMsg)
      {
        if (imageMsg)
          OnImage(imageMsg);
      },
      imageTransport, rclcpp::SensorDataQoS().keep_last(1).get_rmw_qos_profile());

  RCLCPP_INFO(m_node.get_logger(), "Started checkerboard detector");

  return true;
}

void CheckerboardDetectorNode::Deinitialize()
{
  if (m_imageSubscriber)
    m_imageSubscriber->shutdown();

  if (m_debugPublisher)
    m_debugPublisher->shutdown();

  m_statusPublisher.reset();
  m_detector.reset();
  m_frameCount = 0;
}

void CheckerboardDetectorNode::OnImage(const sensor_msgs::msg::Image::ConstSharedPtr& imageMsg)
{
  ++m_frameCount;
  if ((m_frameCount - 1) % static_cast<uint64_t>(m_processingInterval) != 0)
    return;

  cv_bridge::CvImageConstPtr cvImagePtr;
  try
  {
    cvImagePtr = cv_bridge::toCvShare(*imageMsg, imageMsg, imageMsg->encoding);
  }
  catch (const cv_bridge::Exception& e)
  {
    RCLCPP_WARN_THROTTLE(m_node.get_logger(), *m_node.get_clock(), 5000, "cv_bridge exception: %s",
                         e.what());
    PublishStatus(false);
    return;
  }

  if (!cvImagePtr || cvImagePtr->image.empty())
  {
    RCLCPP_WARN_THROTTLE(m_node.get_logger(), *m_node.get_clock(), 5000,
                         "Received empty image frame");
    PublishStatus(false);
    return;
  }

  cv::Mat gray;
  try
  {
    gray = ConvertToGray(cvImagePtr);
  }
  catch (const std::exception& e)
  {
    RCLCPP_WARN_THROTTLE(m_node.get_logger(), *m_node.get_clock(), 5000,
                         "Failed to convert image to grayscale: %s", e.what());
    PublishStatus(false);
    return;
  }

  const CALIBRATION::CheckerboardDetection detection = m_detector->Detect(gray);
  PublishStatus(detection.found);

  if (!m_publishDebugImage || !m_debugPublisher || m_debugPublisher->getNumSubscribers() == 0)
    return;

  cv_bridge::CvImage debugImage;
  debugImage.header = imageMsg->header;
  debugImage.encoding = sensor_msgs::image_encodings::BGR8;

  try
  {
    cv_bridge::CvImagePtr colorImage =
        cv_bridge::toCvCopy(imageMsg, sensor_msgs::image_encodings::BGR8);
    debugImage.image = colorImage->image;
  }
  catch (const cv_bridge::Exception&)
  {
    cv::cvtColor(gray, debugImage.image, cv::COLOR_GRAY2BGR);
  }

  const auto& options = m_detector->Options();
  const cv::Size patternSize{options.checkerboardWidth, options.checkerboardHeight};
  cv::drawChessboardCorners(debugImage.image, patternSize, detection.corners, detection.found);
  m_debugPublisher->publish(debugImage.toImageMsg());
}

void CheckerboardDetectorNode::PublishStatus(bool found)
{
  if (!m_statusPublisher)
    return;

  std_msgs::msg::Bool statusMsg;
  statusMsg.data = found;
  m_statusPublisher->publish(statusMsg);
}
} // namespace OASIS
