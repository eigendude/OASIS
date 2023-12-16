/*
 *  Copyright (C) 2021-2023 Garrett Brown
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
#include <rclcpp/node.hpp>
#include <rcutils/logging_macros.h>
#include <sensor_msgs/image_encodings.hpp>

using namespace OASIS;
using namespace IMAGE;

BackgroundModelerABL::BackgroundModelerABL(std::shared_ptr<rclcpp::Node> node,
                                           const std::string& imageTopic,
                                           const std::string& backgroundTopic)
  : m_logger(node->get_logger()),
    m_imgTransport(std::make_unique<image_transport::ImageTransport>(node)),
    m_imgPublisherBackground(std::make_unique<image_transport::Publisher>()),
    m_imgSubscriber(std::make_unique<image_transport::Subscriber>()),
    m_bgsPackageABL(std::make_unique<bgslibrary::algorithms::AdaptiveBackgroundLearning>())
{
  RCLCPP_INFO(m_logger, "Image topic: %s", imageTopic.c_str());
  RCLCPP_INFO(m_logger, "Background topic: %s", backgroundTopic.c_str());

  auto transportHints = image_transport::TransportHints(node.get(), "compressed");

  *m_imgSubscriber = m_imgTransport->subscribe(imageTopic, 1, &BackgroundModelerABL::ReceiveImage,
                                               this, &transportHints);

  *m_imgPublisherBackground = m_imgTransport->advertise(backgroundTopic, 10);

  RCLCPP_INFO(m_logger, "Started background modeler");
}

BackgroundModelerABL::~BackgroundModelerABL() = default;

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
