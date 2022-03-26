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

namespace
{
constexpr const char* ROS_NAMESPACE = "oasis"; // TODO

// TODO: Hardware configuration
constexpr const char* VIDEO_MACHINE = "netbook";

// Subscribed topics
constexpr const char* IMAGE_TOPIC = "image_raw";

// Published topics
constexpr const char* FOREGROUND_TOPIC = "foreground";
constexpr const char* BACKGROUND_TOPIC = "background";
} // namespace

BackgroundModeler::BackgroundModeler(std::shared_ptr<rclcpp::Node> node)
  : m_logger(node->get_logger()),
    m_imgTransport(std::make_unique<image_transport::ImageTransport>(node)),
    m_imgPublisherForeground(std::make_unique<image_transport::Publisher>()),
    m_imgPublisherBackground(std::make_unique<image_transport::Publisher>()),
    m_imgSubscriber(std::make_unique<image_transport::Subscriber>()),
    m_bgsPackage(std::make_unique<bgslibrary::algorithms::AdaptiveSelectiveBackgroundLearning>())
{
  // Create topics
  const std::string topicBase = std::string("/") + ROS_NAMESPACE + "/" + VIDEO_MACHINE + "/";

  const std::string imageTopic = topicBase + IMAGE_TOPIC;
  const std::string foregroundTopic = topicBase + FOREGROUND_TOPIC;
  const std::string backgroundTopic = topicBase + BACKGROUND_TOPIC;

  RCLCPP_INFO(m_logger, "Image topic: %s", imageTopic.c_str());
  RCLCPP_INFO(m_logger, "Foreground topic: %s", foregroundTopic.c_str());
  RCLCPP_INFO(m_logger, "Background topic: %s", backgroundTopic.c_str());

  auto transportHints = image_transport::TransportHints(node.get(), "compressed");

  *m_imgSubscriber = m_imgTransport->subscribe(imageTopic, 1, &BackgroundModeler::ReceiveImage,
                                               this, &transportHints);

  *m_imgPublisherForeground = m_imgTransport->advertise(foregroundTopic, 10);
  *m_imgPublisherBackground = m_imgTransport->advertise(backgroundTopic, 10);

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

  cv::Mat img_mask;
  cv::Mat img_bkgmodel;

  m_bgsPackage->process(cv_ptr->image, img_mask, img_bkgmodel);

  if (!img_bkgmodel.empty())
  {
    img_bkgmodel.copyTo(cv_ptr->image);
    m_imgPublisherBackground->publish(cv_ptr->toImageMsg());
  }

  if (!img_mask.empty())
  {
    img_mask.copyTo(cv_ptr->image);
    cv_ptr->encoding = "mono8";
    m_imgPublisherForeground->publish(cv_ptr->toImageMsg());
  }
}
