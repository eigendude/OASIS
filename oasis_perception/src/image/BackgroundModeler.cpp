/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "BackgroundModeler.h"

#include <bgslibrary/algorithms/AdaptiveSelectiveBackgroundLearning.h>
#include <bgslibrary/algorithms/IBGS.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <image_transport/transport_hints.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/node.hpp>
#include <rcutils/logging_macros.h>
#include <sensor_msgs/image_encodings.hpp>

using namespace OASIS;
using namespace IMAGE;

BackgroundModeler::BackgroundModeler(std::shared_ptr<rclcpp::Node> node,
                                     const std::string& imageTopic,
                                     const std::string& foregroundTopic,
                                     const std::string& backgroundTopic,
                                     const std::string& subtractedTopic)
  : m_logger(node->get_logger()),
    m_imgTransport(std::make_unique<image_transport::ImageTransport>(node)),
    m_imgPublisherForeground(std::make_unique<image_transport::Publisher>()),
    m_imgPublisherBackground(std::make_unique<image_transport::Publisher>()),
    m_imgPublisherSubtracted(std::make_unique<image_transport::Publisher>()),
    m_imgSubscriber(std::make_unique<image_transport::Subscriber>()),
    m_bgsPackage(std::make_unique<bgslibrary::algorithms::AdaptiveSelectiveBackgroundLearning>())
{
  RCLCPP_INFO(m_logger, "Image topic: %s", imageTopic.c_str());
  RCLCPP_INFO(m_logger, "Foreground topic: %s", foregroundTopic.c_str());
  RCLCPP_INFO(m_logger, "Background topic: %s", backgroundTopic.c_str());
  RCLCPP_INFO(m_logger, "Subtracted topic: %s", subtractedTopic.c_str());

  auto transportHints = image_transport::TransportHints(node.get(), "compressed");

  *m_imgSubscriber = m_imgTransport->subscribe(imageTopic, 1, &BackgroundModeler::ReceiveImage,
                                               this, &transportHints);

  *m_imgPublisherForeground = m_imgTransport->advertise(foregroundTopic, 10);
  *m_imgPublisherBackground = m_imgTransport->advertise(backgroundTopic, 10);
  *m_imgPublisherSubtracted = m_imgTransport->advertise(subtractedTopic, 10);

  RCLCPP_INFO(m_logger, "Started background modeler");
}

BackgroundModeler::~BackgroundModeler() = default;

void BackgroundModeler::ReceiveImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
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

  cv::Mat image = cv_ptr->image.clone();
  cv::Mat imageMask;
  cv::Mat imageBackgroundModel;
  cv::Mat imageSubtracted;

  m_bgsPackage->process(cv_ptr->image, imageMask, imageBackgroundModel);

  if (!imageBackgroundModel.empty())
  {
    imageBackgroundModel.copyTo(cv_ptr->image);
    m_imgPublisherBackground->publish(cv_ptr->toImageMsg());
  }

  if (!imageMask.empty())
  {
    image.copyTo(cv_ptr->image, imageMask);
    m_imgPublisherSubtracted->publish(cv_ptr->toImageMsg());

    imageMask.copyTo(cv_ptr->image);
    cv_ptr->encoding = "mono8";
    m_imgPublisherForeground->publish(cv_ptr->toImageMsg());
  }
}
