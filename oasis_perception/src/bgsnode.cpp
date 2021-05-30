/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of Oasis - https://github.com/eigendude/oasis
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include <memory>
#include <opencv2/imgproc/imgproc.hpp>

#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "image_transport/transport_hints.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "algorithms/AdaptiveSelectiveBackgroundLearning.h"
#include "algorithms/IBGS.h"

constexpr const char *NODE_NAME = "background_subtractor";
constexpr const char *IMAGE_TOPIC = "/oasis/netbook/image_raw"; // TODO

class BackgroundModeller
{
public:
  BackgroundModeller(rclcpp::Node::SharedPtr node)
   : m_node(std::move(node)),
     m_imgTransport(m_node),
     m_bgsPackage(std::make_unique<bgslibrary::algorithms::AdaptiveSelectiveBackgroundLearning>())
  {
    auto transportHints = image_transport::TransportHints(m_node.get(), "compressed");

    m_imgSubscriber = m_imgTransport.subscribe(
      IMAGE_TOPIC,
      1,
      &BackgroundModeller::ReceiveImage,
      this,
      &transportHints
    );

    m_imgPublisherForeground = m_imgTransport.advertise("foreground", 1);
    m_imgPublisherBackground = m_imgTransport.advertise("background", 1);
  }

  ~BackgroundModeller(void) = default;

  void ReceiveImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(m_node->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat img_mask;
    cv::Mat img_bkgmodel;

    m_bgsPackage->process(cv_ptr->image, img_mask, img_bkgmodel);

    if (!img_bkgmodel.empty())
    {
      img_bkgmodel.copyTo(cv_ptr->image);
      m_imgPublisherBackground.publish(cv_ptr->toImageMsg());
    }

    if (!img_mask.empty())
    {
      img_mask.copyTo(cv_ptr->image);
      cv_ptr->encoding = "mono8";
      m_imgPublisherForeground.publish(cv_ptr->toImageMsg());
    }
  }

private:
  rclcpp::Node::SharedPtr m_node;
  image_transport::ImageTransport m_imgTransport;
  image_transport::Subscriber m_imgSubscriber;
  image_transport::Publisher m_imgPublisherForeground;
  image_transport::Publisher m_imgPublisherBackground;
  std::unique_ptr<bgslibrary::algorithms::IBGS> m_bgsPackage;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>(NODE_NAME);

  {
    BackgroundModeller backgroundModeller(node);
    rclcpp::spin(node);
  }

  rclcpp::shutdown();

  return 0;
}
