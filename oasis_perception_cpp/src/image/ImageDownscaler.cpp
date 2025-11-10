################################################################################
#
#  Copyright (C) 2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

#include "ImageDownscaler.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
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
                                 int maxHeight)
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
      m_node.get(), imageTopic,
      [this](const sensor_msgs::msg::Image::ConstSharedPtr& msg) { ReceiveImage(msg); }, imageTransport);
}

ImageDownscaler::~ImageDownscaler()
{
  m_imageSubscriber->shutdown();
  m_downscaledPublisher->shutdown();
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
    m_downscaledPublisher->publish(msg);
    return;
  }

  const double widthScale = static_cast<double>(m_maxWidth) / static_cast<double>(width);
  const double heightScale = static_cast<double>(m_maxHeight) / static_cast<double>(height);
  const double scale = std::min(widthScale, heightScale);

  if (scale >= 1.0 - SCALE_EPSILON)
  {
    m_downscaledPublisher->publish(msg);
    return;
  }

  const int targetWidth = std::max(1, static_cast<int>(std::round(width * scale)));
  const int targetHeight = std::max(1, static_cast<int>(std::round(height * scale)));

  cv::Mat resizedImage;
  cv::resize(image, resizedImage, cv::Size(targetWidth, targetHeight), 0.0, 0.0, cv::INTER_AREA);

  cv_bridge::CvImage downscaledImage(cv_ptr->header, cv_ptr->encoding, resizedImage);
  m_downscaledPublisher->publish(downscaledImage.toImageMsg());
}

