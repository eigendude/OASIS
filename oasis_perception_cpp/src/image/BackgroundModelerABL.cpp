/*
 *  Copyright (C) 2021-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "BackgroundModelerABL.h"

#include <bgslibrary/algorithms/AdaptiveBackgroundLearning.h>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/transport_hints.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/image_encodings.hpp>

using namespace OASIS;
using namespace IMAGE;

BackgroundModelerABL::BackgroundModelerABL(rclcpp::Node& node,
                                           const std::string& imageTopic,
                                           const std::string& backgroundTopic)
  : m_logger(node.get_logger()),
    m_imgPublisherBackground(std::make_unique<image_transport::Publisher>()),
    m_imgSubscriber(std::make_unique<image_transport::Subscriber>()),
    m_bgsPackageABL(std::make_unique<bgslibrary::algorithms::AdaptiveBackgroundLearning>())
{
  *m_imgPublisherBackground = image_transport::create_publisher(&node, backgroundTopic);
  *m_imgSubscriber = image_transport::create_subscription(
      &node, imageTopic, [this](const auto& msg) { ReceiveImage(msg); }, "zstd");
}

BackgroundModelerABL::~BackgroundModelerABL()
{
  m_imgSubscriber->shutdown();
  m_imgPublisherBackground->shutdown();
}

void BackgroundModelerABL::ReceiveImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(m_logger, "cv_bridge exception: %s", e.what());
    return;
  }

  cv_bridge::CvImage imageMaskABL(cv_ptr->header, sensor_msgs::image_encodings::MONO8);
  cv_bridge::CvImage imageBackgroundModelABL(cv_ptr->header, cv_ptr->encoding);

  m_bgsPackageABL->process(cv_ptr->image, imageMaskABL.image, imageBackgroundModelABL.image);

  // Use ABL for the background (ASBL doesn't provide a background)
  if (!imageBackgroundModelABL.image.empty())
    m_imgPublisherBackground->publish(imageBackgroundModelABL.toImageMsg());
}
