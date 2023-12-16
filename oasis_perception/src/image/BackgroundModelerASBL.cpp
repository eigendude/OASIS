/*
 *  Copyright (C) 2021-2023 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "BackgroundModelerASBL.h"

#include <bgslibrary/algorithms/AdaptiveSelectiveBackgroundLearning.h>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/transport_hints.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/node.hpp>
#include <rcutils/logging_macros.h>
#include <sensor_msgs/image_encodings.hpp>

using namespace OASIS;
using namespace IMAGE;

BackgroundModelerASBL::BackgroundModelerASBL(std::shared_ptr<rclcpp::Node> node,
                                             const std::string& imageTopic,
                                             const std::string& foregroundTopic,
                                             const std::string& subtractedTopic)
  : m_logger(node->get_logger()),
    m_imgTransport(std::make_unique<image_transport::ImageTransport>(node)),
    m_imgPublisherForeground(std::make_unique<image_transport::Publisher>()),
    m_imgPublisherSubtracted(std::make_unique<image_transport::Publisher>()),
    m_imgSubscriber(std::make_unique<image_transport::Subscriber>()),
    m_bgsPackageASBL(
        std::make_unique<bgslibrary::algorithms::AdaptiveSelectiveBackgroundLearning>())
{
  RCLCPP_INFO(m_logger, "Image topic: %s", imageTopic.c_str());
  RCLCPP_INFO(m_logger, "Foreground topic: %s", foregroundTopic.c_str());
  RCLCPP_INFO(m_logger, "Subtracted topic: %s", subtractedTopic.c_str());

  auto transportHints = image_transport::TransportHints(node.get(), "compressed");

  *m_imgSubscriber = m_imgTransport->subscribe(imageTopic, 1, &BackgroundModelerASBL::ReceiveImage,
                                               this, &transportHints);

  *m_imgPublisherForeground = m_imgTransport->advertise(foregroundTopic, 10);
  *m_imgPublisherSubtracted = m_imgTransport->advertise(subtractedTopic, 10);

  RCLCPP_INFO(m_logger, "Started background modeler");
}

BackgroundModelerASBL::~BackgroundModelerASBL() = default;

void BackgroundModelerASBL::ReceiveImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
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
