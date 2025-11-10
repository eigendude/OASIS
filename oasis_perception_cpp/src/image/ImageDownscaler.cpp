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
                                 int maxWidth,
                                 int maxHeight,
                                 const std::string& cameraInfoTopic,
                                 const std::string& downscaledCameraInfoTopic)
  : m_logger(node->get_logger()),
    m_node(std::move(node)),
    m_downscaledPublisher(std::make_unique<image_transport::Publisher>()),
    m_imageSubscriber(std::make_unique<image_transport::Subscriber>()),
    m_maxWidth(maxWidth),
    m_maxHeight(maxHeight)
{
  if (!m_node)
  {
    throw std::invalid_argument("ImageDownscaler requires a valid node");
  }

  if (m_maxWidth <= 0 || m_maxHeight <= 0)
  {
    throw std::invalid_argument("ImageDownscaler requires positive maximum dimensions");
  }

  *m_downscaledPublisher = image_transport::create_publisher(m_node.get(), downscaledTopic);
  *m_imageSubscriber = image_transport::create_subscription(
      m_node.get(), imageTopic, [this](const sensor_msgs::msg::Image::ConstSharedPtr& msg)
      { ReceiveImage(msg); }, imageTransport);

  m_cameraInfoPublisher = m_node->create_publisher<sensor_msgs::msg::CameraInfo>(
      downscaledCameraInfoTopic, rclcpp::SensorDataQoS());

  m_cameraInfoSubscriber = m_node->create_subscription<sensor_msgs::msg::CameraInfo>(
      cameraInfoTopic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) { ReceiveCameraInfo(msg); });
}

ImageDownscaler::~ImageDownscaler()
{
  m_imageSubscriber->shutdown();
  m_downscaledPublisher->shutdown();
  if (m_cameraInfoSubscriber)
    m_cameraInfoSubscriber.reset();
  if (m_cameraInfoPublisher)
    m_cameraInfoPublisher.reset();
}

void ImageDownscaler::ReceiveImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  if (!msg)
  {
    RCLCPP_WARN(m_logger, "Received null image message");
    return;
  }

  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
  }
  catch (const cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(m_logger, "cv_bridge exception: %s", e.what());
    return;
  }

  const cv::Mat& image = cv_ptr->image;
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

  if (width <= m_maxWidth && height <= m_maxHeight)
  {
    PublishDownscaledCameraInfo(msg->header, width, height, width, height);
    m_downscaledPublisher->publish(msg);
    return;
  }

  const double widthScale = static_cast<double>(m_maxWidth) / static_cast<double>(width);
  const double heightScale = static_cast<double>(m_maxHeight) / static_cast<double>(height);
  const double scale = std::min(widthScale, heightScale);

  if (scale >= 1.0 - SCALE_EPSILON)
  {
    PublishDownscaledCameraInfo(msg->header, width, height, width, height);
    m_downscaledPublisher->publish(msg);
    return;
  }

  const int targetWidth = std::max(1, static_cast<int>(std::round(width * scale)));
  const int targetHeight = std::max(1, static_cast<int>(std::round(height * scale)));

  cv::Mat resizedImage;
  cv::resize(image, resizedImage, cv::Size(targetWidth, targetHeight), 0.0, 0.0, cv::INTER_AREA);

  cv_bridge::CvImage downscaledImage(cv_ptr->header, cv_ptr->encoding, resizedImage);
  auto downscaledMessage = downscaledImage.toImageMsg();
  PublishDownscaledCameraInfo(downscaledMessage->header, width, height, targetWidth, targetHeight);
  m_downscaledPublisher->publish(downscaledMessage);
}

void ImageDownscaler::ReceiveCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr& msg)
{
  if (!msg)
  {
    RCLCPP_WARN(m_logger, "Received null camera info message");
    return;
  }

  std::scoped_lock lock(m_cameraInfoMutex);
  m_lastCameraInfo = std::make_shared<sensor_msgs::msg::CameraInfo>(*msg);
}

void ImageDownscaler::PublishDownscaledCameraInfo(const std_msgs::msg::Header& header,
                                                  int originalWidth,
                                                  int originalHeight,
                                                  int outputWidth,
                                                  int outputHeight)
{
  sensor_msgs::msg::CameraInfo::SharedPtr cameraInfo;
  {
    std::scoped_lock lock(m_cameraInfoMutex);
    if (!m_lastCameraInfo)
    {
      // Cannot publish camera info before receiving any camera info messages
      return;
    }

    cameraInfo = std::make_shared<sensor_msgs::msg::CameraInfo>(*m_lastCameraInfo);
  }

  cameraInfo->header.stamp = header.stamp;
  if (!header.frame_id.empty())
    cameraInfo->header.frame_id = header.frame_id;

  const bool shouldScale = (outputWidth != originalWidth) || (outputHeight != originalHeight);
  if (shouldScale && originalWidth > 0 && originalHeight > 0)
  {
    const double widthScale = static_cast<double>(outputWidth) / static_cast<double>(originalWidth);
    const double heightScale =
        static_cast<double>(outputHeight) / static_cast<double>(originalHeight);
    const double scale = std::min(widthScale, heightScale);

    cameraInfo->width = static_cast<uint32_t>(std::max(1, outputWidth));
    cameraInfo->height = static_cast<uint32_t>(std::max(1, outputHeight));

    cameraInfo->k[0] *= scale; // fx
    cameraInfo->k[2] *= scale; // cx
    cameraInfo->k[4] *= scale; // fy
    cameraInfo->k[5] *= scale; // cy

    cameraInfo->p[0] *= scale; // fx
    cameraInfo->p[2] *= scale; // cx
    cameraInfo->p[5] *= scale; // fy
    cameraInfo->p[6] *= scale; // cy

    if (cameraInfo->roi.width > 0 && cameraInfo->roi.height > 0)
    {
      cameraInfo->roi.x_offset = static_cast<uint32_t>(std::max(
          0, static_cast<int>(std::lround(static_cast<double>(cameraInfo->roi.x_offset) * scale))));
      cameraInfo->roi.y_offset = static_cast<uint32_t>(std::max(
          0, static_cast<int>(std::lround(static_cast<double>(cameraInfo->roi.y_offset) * scale))));
      cameraInfo->roi.width = static_cast<uint32_t>(std::max(
          1, static_cast<int>(std::lround(static_cast<double>(cameraInfo->roi.width) * scale))));
      cameraInfo->roi.height = static_cast<uint32_t>(std::max(
          1, static_cast<int>(std::lround(static_cast<double>(cameraInfo->roi.height) * scale))));
    }
  }
  else
  {
    cameraInfo->width = static_cast<uint32_t>(std::max(1, outputWidth));
    cameraInfo->height = static_cast<uint32_t>(std::max(1, outputHeight));
  }

  m_cameraInfoPublisher->publish(*cameraInfo);
}
