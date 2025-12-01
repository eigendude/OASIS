/*
 *  Copyright (C) 2021-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "BackgroundSubtractorASBL.h"

#include <bgslibrary/algorithms/AdaptiveSelectiveBackgroundLearning.h>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/transport_hints.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/image_encodings.hpp>

using namespace OASIS;
using namespace IMAGE;

BackgroundSubtractorASBL::BackgroundSubtractorASBL(rclcpp::Node& node,
                                                   const std::string& imageTopic,
                                                   const std::string& foregroundTopic,
                                                   const std::string& subtractedTopic)
  : m_logger(node.get_logger()),
    m_imgPublisherForeground(std::make_unique<image_transport::Publisher>()),
    m_imgPublisherSubtracted(std::make_unique<image_transport::Publisher>()),
    m_imgSubscriber(std::make_unique<image_transport::Subscriber>()),
    m_bgsPackageASBL(
        std::make_unique<bgslibrary::algorithms::AdaptiveSelectiveBackgroundLearning>())
{
  *m_imgPublisherForeground = image_transport::create_publisher(&node, foregroundTopic);
  *m_imgPublisherSubtracted = image_transport::create_publisher(&node, subtractedTopic);
  *m_imgSubscriber = image_transport::create_subscription(
      &node, imageTopic, [this](const auto& msg) { ReceiveImage(msg); }, "compressed",
      rclcpp::QoS{1}.get_rmw_qos_profile());
}

BackgroundSubtractorASBL::~BackgroundSubtractorASBL()
{
  m_imgSubscriber->shutdown();
  m_imgPublisherSubtracted->shutdown();
  m_imgPublisherForeground->shutdown();
}

void BackgroundSubtractorASBL::ReceiveImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
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

  cv_bridge::CvImage imageMaskASBL(cv_ptr->header, sensor_msgs::image_encodings::MONO8);
  cv_bridge::CvImage imageBackgroundModelASBL(cv_ptr->header, cv_ptr->encoding);
  cv_bridge::CvImage imageSubtractedASBL(cv_ptr->header, cv_ptr->encoding);

  m_bgsPackageASBL->process(cv_ptr->image, imageMaskASBL.image, imageBackgroundModelASBL.image);

  // Use ASBL for the foreground
  if (!imageMaskASBL.image.empty())
  {
    m_imgPublisherForeground->publish(imageMaskASBL.toImageMsg());

    // Publish the subtracted image
    cv_ptr->image.copyTo(imageSubtractedASBL.image, imageMaskASBL.image);
    m_imgPublisherSubtracted->publish(imageSubtractedASBL.toImageMsg());
  }
}
