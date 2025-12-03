/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "ImageDownscaler.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/image_encodings.hpp>

using namespace OASIS;
using namespace IMAGE;

namespace
{
constexpr double SCALE_EPSILON = 1e-6;
} // namespace

ImageDownscaler::ImageDownscaler(std::shared_ptr<rclcpp::Node> node,
                                 const std::string& imageTopic,
                                 const std::string& downscaledTopic,
                                 const std::string& imageTransport,
                                 std::optional<unsigned int> outputWidth,
                                 std::optional<unsigned int> outputHeight,
                                 std::optional<unsigned int> maxWidth,
                                 std::optional<unsigned int> maxHeight)
  : m_logger(node->get_logger()),
    m_node(std::move(node)),
    m_downscaledPublisher(std::make_unique<image_transport::CameraPublisher>()),
    m_cameraSubscriber(std::make_unique<image_transport::CameraSubscriber>())
{
  if (!m_node)
  {
    throw std::invalid_argument("ImageDownscaler requires a valid node");
  }

  if (outputWidth && *outputWidth == 0)
  {
    throw std::invalid_argument("ImageDownscaler requires a positive output width");
  }

  if (outputHeight && *outputHeight == 0)
  {
    throw std::invalid_argument("ImageDownscaler requires a positive output height");
  }

  if (maxWidth && *maxWidth == 0)
  {
    throw std::invalid_argument("ImageDownscaler requires a positive maximum width");
  }

  if (maxHeight && *maxHeight == 0)
  {
    throw std::invalid_argument("ImageDownscaler requires a positive maximum height");
  }

  if (!outputWidth && !outputHeight && !maxWidth && !maxHeight)
  {
    RCLCPP_WARN(m_logger,
                "No sizing parameters provided to image downscaler. Incoming images will be "
                "republished without resizing.");
  }

  m_outputWidth = outputWidth;
  m_outputHeight = outputHeight;
  m_maxWidth = maxWidth;
  m_maxHeight = maxHeight;

  // Publishers
  *m_downscaledPublisher = image_transport::create_camera_publisher(m_node.get(), downscaledTopic);

  // Subscribers
  *m_cameraSubscriber = image_transport::create_camera_subscription(
      m_node.get(), imageTopic,
      [this](const sensor_msgs::msg::Image::ConstSharedPtr& imageMsg,
             const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cameraInfo)
      { ReceiveImage(imageMsg, cameraInfo); },
      imageTransport, rclcpp::QoS{1}.get_rmw_qos_profile());
}

ImageDownscaler::~ImageDownscaler()
{
  // Deinitialize subscribers
  m_cameraSubscriber->shutdown();

  // Deinitialize publishers
  m_downscaledPublisher->shutdown();
}

void ImageDownscaler::ReceiveImage(
    const sensor_msgs::msg::Image::ConstSharedPtr& imageMsg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cameraInfoPtr)
{
  if (!imageMsg)
  {
    RCLCPP_WARN(m_logger, "Received null image message");
    return;
  }

  if (!cameraInfoPtr)
  {
    RCLCPP_WARN(m_logger, "Received null camera info message");
    return;
  }

  //
  // Get downscaled image
  //

  cv_bridge::CvImageConstPtr cvImagePtr;
  try
  {
    cvImagePtr = cv_bridge::toCvShare(imageMsg, imageMsg->encoding);
  }
  catch (const cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(m_logger, "cv_bridge exception: %s", e.what());
    return;
  }

  const cv::Mat& image = cvImagePtr->image;
  if (image.empty())
  {
    RCLCPP_WARN(m_logger, "Received empty image frame");
    return;
  }

  const int width = image.cols;
  const int height = image.rows;

  if (width <= 0 || height <= 0)
  {
    RCLCPP_WARN(m_logger, "Invalid image dimensions: %dx%d", width, height);
    return;
  }

  const auto [targetWidth, targetHeight] = CalculateTargetDimensions(width, height);

  if (targetWidth == static_cast<unsigned int>(width) &&
      targetHeight == static_cast<unsigned int>(height))
  {
    m_downscaledPublisher->publish(imageMsg, cameraInfoPtr);
    return;
  }

  cv::Mat resizedImage;
  cv::resize(image, resizedImage, cv::Size(targetWidth, targetHeight), 0.0, 0.0, cv::INTER_AREA);

  const cv_bridge::CvImage downscaledImage(cvImagePtr->header, cvImagePtr->encoding, resizedImage);
  const sensor_msgs::msg::Image::SharedPtr downscaledMessage = downscaledImage.toImageMsg();

  //
  // Get downscaled camera info
  //

  sensor_msgs::msg::CameraInfo cameraInfo = *cameraInfoPtr;

  const unsigned int originalWidth = static_cast<unsigned int>(cameraInfo.width);
  const unsigned int originalHeight = static_cast<unsigned int>(cameraInfo.height);

  if (originalWidth == 0 || originalHeight == 0)
  {
    RCLCPP_ERROR(m_logger, "Invalid camera info dimensions: %dx%d", originalWidth, originalHeight);
    return;
  }

  const auto [outputWidth, outputHeight] = CalculateTargetDimensions(originalWidth, originalHeight);

  const bool shouldScale = (outputWidth != originalWidth) || (outputHeight != originalHeight);
  if (shouldScale)
  {
    cameraInfo.width = static_cast<uint32_t>(std::max(1U, outputWidth));
    cameraInfo.height = static_cast<uint32_t>(std::max(1U, outputHeight));

    const double widthScale = static_cast<double>(outputWidth) / static_cast<double>(originalWidth);
    const double heightScale =
        static_cast<double>(outputHeight) / static_cast<double>(originalHeight);

    // Scale K matrix
    cameraInfo.k[0] *= widthScale; // fx
    cameraInfo.k[2] *= widthScale; // cx
    cameraInfo.k[4] *= heightScale; // fy
    cameraInfo.k[5] *= heightScale; // cy

    // Scale P matrix
    cameraInfo.p[0] *= widthScale; // fx
    cameraInfo.p[2] *= widthScale; // cx
    cameraInfo.p[5] *= heightScale; // fy
    cameraInfo.p[6] *= heightScale; // cy

    if (cameraInfo.roi.width > 0 && cameraInfo.roi.height > 0)
    {
      cameraInfo.roi.x_offset = static_cast<uint32_t>(
          std::max(0U, static_cast<unsigned int>(std::lround(
                           static_cast<double>(cameraInfo.roi.x_offset) * widthScale))));
      cameraInfo.roi.y_offset = static_cast<uint32_t>(
          std::max(0U, static_cast<unsigned int>(std::lround(
                           static_cast<double>(cameraInfo.roi.y_offset) * heightScale))));
      cameraInfo.roi.width = static_cast<uint32_t>(
          std::max(1U, static_cast<unsigned int>(
                           std::lround(static_cast<double>(cameraInfo.roi.width) * widthScale))));
      cameraInfo.roi.height = static_cast<uint32_t>(
          std::max(1U, static_cast<unsigned int>(
                           std::lround(static_cast<double>(cameraInfo.roi.height) * heightScale))));
    }
  }
  else
  {
    cameraInfo.width = static_cast<uint32_t>(std::max(1U, outputWidth));
    cameraInfo.height = static_cast<uint32_t>(std::max(1U, outputHeight));
  }

  //
  // Publish image/camera info pair
  //

  m_downscaledPublisher->publish(
      downscaledMessage, std::make_shared<sensor_msgs::msg::CameraInfo>(std::move(cameraInfo)));
}

std::pair<unsigned int, unsigned int> ImageDownscaler::CalculateTargetDimensions(
    unsigned int width, unsigned int height) const
{
  if (width == 0 || height == 0)
    return {width, height};

  if (m_outputWidth || m_outputHeight)
  {
    unsigned int targetWidth = width;
    unsigned int targetHeight = height;

    if (m_outputWidth && m_outputHeight)
    {
      targetWidth = *m_outputWidth;
      targetHeight = *m_outputHeight;
    }
    else if (m_outputWidth)
    {
      targetWidth = *m_outputWidth;
      const double aspectRatio = static_cast<double>(width) / static_cast<double>(height);
      targetHeight = std::max(1U, static_cast<unsigned int>(
                                      std::lround(static_cast<double>(targetWidth) / aspectRatio)));
    }
    else if (m_outputHeight)
    {
      targetHeight = *m_outputHeight;
      const double aspectRatio = static_cast<double>(width) / static_cast<double>(height);
      targetWidth = std::max(1U, static_cast<unsigned int>(
                                     std::lround(static_cast<double>(targetHeight) * aspectRatio)));
    }

    return {targetWidth, targetHeight};
  }

  if (!m_maxWidth && !m_maxHeight)
    return {width, height};

  const double widthScale =
      m_maxWidth ? static_cast<double>(*m_maxWidth) / static_cast<double>(width) : 1.0;
  const double heightScale =
      m_maxHeight ? static_cast<double>(*m_maxHeight) / static_cast<double>(height) : 1.0;
  const double scale = std::min(widthScale, heightScale);

  if (scale >= 1.0 - SCALE_EPSILON)
    return {width, height};

  const unsigned int targetWidth =
      std::max(1U, static_cast<unsigned int>(std::round(width * scale)));
  const unsigned int targetHeight =
      std::max(1U, static_cast<unsigned int>(std::round(height * scale)));

  return {targetWidth, targetHeight};
}
