/*
 *  Copyright (C) 2022 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <atomic>
#include <map>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/logger.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace bgslibrary
{
namespace algorithms
{
class IBGS;
} // namespace algorithms
} // namespace bgslibrary

namespace image_transport
{
class ImageTransport;
class Publisher;
class Subscriber;
} // namespace image_transport

namespace rclcpp
{
class Node;
}

namespace OASIS
{
namespace IMAGE
{

class MultiModeler
{
public:
  MultiModeler(std::shared_ptr<rclcpp::Node> node,
               const std::string& imageTopic,
               const std::string& foregroundTopic,
               const std::string& backgroundTopic,
               const std::string& subtractedTopic);
  ~MultiModeler();

  // ROS interface
  void ReceiveImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

private:
  /*!
   * \brief Get topic names for all supported algorithms
   */
  static std::vector<const char*> GetAlgorithms();

  // Logging parameters
  rclcpp::Logger m_logger;

  struct ImagePublisher
  {
    ImagePublisher();
    std::unique_ptr<image_transport::Publisher> foreground;
    std::unique_ptr<image_transport::Publisher> background;
    std::unique_ptr<image_transport::Publisher> subtracted;
  };

  // Image parameters
  std::unique_ptr<image_transport::ImageTransport> m_imgTransport;
  std::map<std::string, ImagePublisher> m_imgPublishers; // Algorithm topic -> publisher
  std::unique_ptr<image_transport::Subscriber> m_imgSubscriber;

  // Basic background subtractors
  std::unique_ptr<bgslibrary::algorithms::IBGS> m_bgsPackageAdaptiveBackgroundLearning;
  std::unique_ptr<bgslibrary::algorithms::IBGS> m_bgsPackageAdaptiveSelectiveBackgroundLearning;
  std::unique_ptr<bgslibrary::algorithms::IBGS>
      m_bgsPackageSigmaDelta; // Manzanera and Richefeu (2004)

  // Non-parametric background subtractors
  std::unique_ptr<bgslibrary::algorithms::IBGS>
      m_bgsPackageKNN; // Zoran Zivkovic and Ferdinand van der Heijden

  struct ImageThread
  {
    std::unique_ptr<std::thread> thread;
    std::atomic<bool> running{false};
  };

  // Threads
  std::map<std::string, ImageThread> m_threads; // Algorithm topic -> thread
};

} // namespace IMAGE
} // namespace OASIS
