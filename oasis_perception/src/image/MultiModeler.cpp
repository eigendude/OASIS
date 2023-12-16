/*
 *  Copyright (C) 2022-2023 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "MultiModeler.h"

#include <bgslibrary/algorithms/AdaptiveBackgroundLearning.h>
#include <bgslibrary/algorithms/AdaptiveSelectiveBackgroundLearning.h>
#include <bgslibrary/algorithms/KNN.h>
#include <bgslibrary/algorithms/SigmaDelta.h>
#include <cv_bridge/cv_bridge.hpp>
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
// Published algorithms
constexpr const char* ABL_TOPIC = "abl"; // AdaptiveBackgroundLearning
constexpr const char* ASBL_TOPIC = "asbl"; // AdaptiveSelectiveBackgroundLearning
constexpr const char* SD_TOPIC = "sd"; // SigmaDelta
constexpr const char* KNN_TOPIC = "knn"; // KNN
} // namespace

MultiModeler::ImagePublisher::ImagePublisher()
  : foreground(std::make_unique<image_transport::Publisher>()),
    background(std::make_unique<image_transport::Publisher>()),
    subtracted(std::make_unique<image_transport::Publisher>())
{
}

MultiModeler::MultiModeler(std::shared_ptr<rclcpp::Node> node,
                           const std::string& imageTopic,
                           const std::string& foregroundTopic,
                           const std::string& backgroundTopic,
                           const std::string& subtractedTopic)
  : m_logger(node->get_logger()),
    m_imgTransport(std::make_unique<image_transport::ImageTransport>(node)),
    m_imgSubscriber(std::make_unique<image_transport::Subscriber>()),
    m_bgsPackageAdaptiveBackgroundLearning(
        std::make_unique<bgslibrary::algorithms::AdaptiveBackgroundLearning>()),
    m_bgsPackageAdaptiveSelectiveBackgroundLearning(
        std::make_unique<bgslibrary::algorithms::AdaptiveSelectiveBackgroundLearning>()),
    m_bgsPackageSigmaDelta(std::make_unique<bgslibrary::algorithms::SigmaDelta>()),
    m_bgsPackageKNN(std::make_unique<bgslibrary::algorithms::KNN>())
{
  auto transportHints = image_transport::TransportHints(node.get(), "compressed");

  *m_imgSubscriber =
      m_imgTransport->subscribe(imageTopic, 1, &MultiModeler::ReceiveImage, this, &transportHints);

  for (const char* algorithm : GetAlgorithms())
  {
    ImagePublisher& publisher = m_imgPublishers[algorithm];

    const std::string foregroundAlgorithm = foregroundTopic + "/" + algorithm;
    const std::string backgroundAlgorithm = backgroundTopic + "/" + algorithm;
    const std::string subtractedAlgorithm = subtractedTopic + "/" + algorithm;

    RCLCPP_INFO(m_logger, "Algorithm:: %s", algorithm);
    RCLCPP_INFO(m_logger, "  Foreground topic: %s", foregroundAlgorithm.c_str());
    RCLCPP_INFO(m_logger, "  Background topic: %s", backgroundAlgorithm.c_str());
    RCLCPP_INFO(m_logger, "  Subtracted topic: %s", subtractedAlgorithm.c_str());

    *publisher.foreground = m_imgTransport->advertise(foregroundAlgorithm, 1, true);
    *publisher.background = m_imgTransport->advertise(backgroundAlgorithm, 1, true);
    *publisher.subtracted = m_imgTransport->advertise(subtractedAlgorithm, 1, true);
  }

  RCLCPP_INFO(m_logger, "Started background modeler");
}

MultiModeler::~MultiModeler()
{
  for (const auto& [topic, thread] : m_threads)
  {
    if (thread.thread)
      thread.thread->join();
  }
}

void MultiModeler::ReceiveImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
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

  // AdaptiveBackgroundLearning
  {
    const char* algorithm = ABL_TOPIC;
    bgslibrary::algorithms::IBGS& bgsPackage = *m_bgsPackageAdaptiveBackgroundLearning;

    ImagePublisher& publisher = m_imgPublishers[algorithm];

    ImageThread& thread = m_threads[algorithm];
    std::atomic<bool>& running = thread.running;

    if (!running)
    {
      if (thread.thread)
        thread.thread->join();

      running = true;

      thread.thread = std::make_unique<std::thread>(
          [this, &running, &bgsPackage, &publisher, cv_ptr]()
          {
            cv::Mat imageMask;
            cv::Mat imageBackgroundModel;

            bgsPackage.process(cv_ptr->image, imageMask, imageBackgroundModel);

            if (!imageMask.empty())
            {
              cv_bridge::CvImagePtr cv_ptrSubtracted =
                  std::make_shared<cv_bridge::CvImage>(cv_ptr->header, cv_ptr->encoding);
              cv_ptr->image.copyTo(cv_ptrSubtracted->image, imageMask);
              publisher.subtracted->publish(cv_ptrSubtracted->toImageMsg());

              cv_bridge::CvImagePtr cv_ptrForeground = std::make_shared<cv_bridge::CvImage>(
                  cv_ptr->header, "mono8", std::move(imageMask));
              publisher.foreground->publish(cv_ptrForeground->toImageMsg());
            }

            if (!imageBackgroundModel.empty())
            {
              cv_bridge::CvImagePtr cv_ptrBackground = std::make_shared<cv_bridge::CvImage>(
                  cv_ptr->header, cv_ptr->encoding, std::move(imageBackgroundModel));
              publisher.background->publish(cv_ptrBackground->toImageMsg());
            }

            running = false;
          });
    }
  }

  // AdaptiveSelectiveBackgroundLearning
  {
    const char* algorithm = ASBL_TOPIC;
    bgslibrary::algorithms::IBGS& bgsPackage = *m_bgsPackageAdaptiveSelectiveBackgroundLearning;

    ImagePublisher& publisher = m_imgPublishers[algorithm];

    ImageThread& thread = m_threads[algorithm];
    std::atomic<bool>& running = thread.running;

    if (!running)
    {
      if (thread.thread)
        thread.thread->join();

      running = true;

      thread.thread = std::make_unique<std::thread>(
          [this, &running, &bgsPackage, &publisher, cv_ptr]()
          {
            cv::Mat imageMask;
            cv::Mat imageBackgroundModel;

            bgsPackage.process(cv_ptr->image, imageMask, imageBackgroundModel);

            if (!imageMask.empty())
            {
              cv_bridge::CvImagePtr cv_ptrSubtracted =
                  std::make_shared<cv_bridge::CvImage>(cv_ptr->header, cv_ptr->encoding);
              cv_ptr->image.copyTo(cv_ptrSubtracted->image, imageMask);
              publisher.subtracted->publish(cv_ptrSubtracted->toImageMsg());

              cv_bridge::CvImagePtr cv_ptrForeground = std::make_shared<cv_bridge::CvImage>(
                  cv_ptr->header, "mono8", std::move(imageMask));
              publisher.foreground->publish(cv_ptrForeground->toImageMsg());
            }

            running = false;
          });
    }
  }

  // SigmaDelta
  {
    const char* algorithm = SD_TOPIC;
    bgslibrary::algorithms::IBGS& bgsPackage = *m_bgsPackageSigmaDelta;

    ImagePublisher& publisher = m_imgPublishers[algorithm];

    ImageThread& thread = m_threads[algorithm];
    std::atomic<bool>& running = thread.running;

    if (!running)
    {
      if (thread.thread)
        thread.thread->join();

      running = true;

      thread.thread = std::make_unique<std::thread>(
          [&running, &bgsPackage, &publisher, cv_ptr]()
          {
            cv::Mat imageMask;
            cv::Mat imageBackgroundModel;

            bgsPackage.process(cv_ptr->image, imageMask, imageBackgroundModel);

            if (!imageMask.empty())
            {
              cv_bridge::CvImagePtr cv_ptrSubtracted =
                  std::make_shared<cv_bridge::CvImage>(cv_ptr->header, cv_ptr->encoding);
              cv_ptr->image.copyTo(cv_ptrSubtracted->image, imageMask);
              publisher.subtracted->publish(cv_ptrSubtracted->toImageMsg());

              cv_bridge::CvImagePtr cv_ptrForeground = std::make_shared<cv_bridge::CvImage>(
                  cv_ptr->header, "mono8", std::move(imageMask));
              publisher.foreground->publish(cv_ptrForeground->toImageMsg());
            }

            if (!imageBackgroundModel.empty())
            {
              cv_bridge::CvImagePtr cv_ptrBackground = std::make_shared<cv_bridge::CvImage>(
                  cv_ptr->header, cv_ptr->encoding, std::move(imageBackgroundModel));
              publisher.background->publish(cv_ptrBackground->toImageMsg());
            }

            running = false;
          });
    }
  }

  // KNN
  {
    const char* algorithm = KNN_TOPIC;
    bgslibrary::algorithms::IBGS& bgsPackage = *m_bgsPackageKNN;

    ImagePublisher& publisher = m_imgPublishers[algorithm];

    ImageThread& thread = m_threads[algorithm];
    std::atomic<bool>& running = thread.running;

    if (!running)
    {
      if (thread.thread)
        thread.thread->join();

      running = true;

      thread.thread = std::make_unique<std::thread>(
          [&running, &bgsPackage, &publisher, cv_ptr]()
          {
            cv::Mat imageMask;
            cv::Mat imageBackgroundModel;

            bgsPackage.process(cv_ptr->image, imageMask, imageBackgroundModel);

            if (!imageMask.empty())
            {
              cv_bridge::CvImagePtr cv_ptrSubtracted =
                  std::make_shared<cv_bridge::CvImage>(cv_ptr->header, cv_ptr->encoding);
              cv_ptr->image.copyTo(cv_ptrSubtracted->image, imageMask);
              publisher.subtracted->publish(cv_ptrSubtracted->toImageMsg());

              cv_bridge::CvImagePtr cv_ptrForeground = std::make_shared<cv_bridge::CvImage>(
                  cv_ptr->header, "mono8", std::move(imageMask));
              publisher.foreground->publish(cv_ptrForeground->toImageMsg());
            }

            if (!imageBackgroundModel.empty())
            {
              cv_bridge::CvImagePtr cv_ptrBackground = std::make_shared<cv_bridge::CvImage>(
                  cv_ptr->header, cv_ptr->encoding, std::move(imageBackgroundModel));
              publisher.background->publish(cv_ptrBackground->toImageMsg());
            }

            running = false;
          });
    }
  }
}

std::vector<const char*> MultiModeler::GetAlgorithms()
{
  return {
      // SFD_TOPIC,
      // FD_TOPIC,
      // WMM_TOPIC,
      // WMV_TOPIC,
      ABL_TOPIC,
      ASBL_TOPIC,
      SD_TOPIC,
      // IMBS_TOPIC,
      KNN_TOPIC,
  };
}
