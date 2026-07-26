/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "calibration/CheckerboardDetector.h"
#include "nodes/CheckerboardDetectorNode.h"

#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace OASIS
{
class CheckerboardDetectorNodeTestAccess
{
public:
  template<typename Callback>
  static void SetDetectionCallback(CheckerboardDetectorNode& detectorNode,
                                   Callback&& detectionCallback)
  {
    detectorNode.m_detectionCallback = std::forward<Callback>(detectionCallback);
  }

  template<typename Callback>
  static void SetDebugImageCallback(CheckerboardDetectorNode& detectorNode,
                                    Callback&& debugImageCallback)
  {
    detectorNode.m_debugImageCallback = std::forward<Callback>(debugImageCallback);
  }

  template<typename Callback>
  static void SetStatusCallback(CheckerboardDetectorNode& detectorNode, Callback&& statusCallback)
  {
    detectorNode.m_statusCallback = std::forward<Callback>(statusCallback);
  }

  static void OnImage(CheckerboardDetectorNode& detectorNode,
                      const sensor_msgs::msg::Image::ConstSharedPtr& imageMessage)
  {
    detectorNode.OnImage(imageMessage);
  }
};
} // namespace OASIS

namespace
{
class CheckerboardDetectorNodeTest : public testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok())
    {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok())
      rclcpp::shutdown();
  }

  static sensor_msgs::msg::Image::ConstSharedPtr CreateMono8Image()
  {
    const cv::Mat image{480, 640, CV_8UC1, cv::Scalar{127}};
    cv_bridge::CvImage cvImage;
    cvImage.encoding = sensor_msgs::image_encodings::MONO8;
    cvImage.image = image;
    return cvImage.toImageMsg();
  }

  static sensor_msgs::msg::Image::ConstSharedPtr CreateImage(const cv::Mat& image,
                                                             const std::string& encoding)
  {
    cv_bridge::CvImage cvImage;
    cvImage.encoding = encoding;
    cvImage.image = image;
    return cvImage.toImageMsg();
  }
};

TEST_F(CheckerboardDetectorNodeTest, DetectionExceptionIsContainedAndLaterFrameIsProcessed)
{
  rclcpp::Node node{"checkerboard_detection_exception_test"};
  OASIS::CheckerboardDetectorNode detectorNode{node};
  std::vector<bool> statuses;
  size_t detectionCallCount = 0;

  OASIS::CheckerboardDetectorNodeTestAccess::SetDetectionCallback(
      detectorNode,
      [&detectionCallCount](const cv::Mat&, OASIS::CALIBRATION::CheckerboardDetection& detection)
      {
        ++detectionCallCount;
        if (detectionCallCount == 1)
          CV_Error(cv::Error::StsAssert, "Injected checkerboard detection failure");

        detection.found = true;
      });
  OASIS::CheckerboardDetectorNodeTestAccess::SetDebugImageCallback(
      detectorNode, [](const sensor_msgs::msg::Image::ConstSharedPtr&, const cv::Mat&,
                       const OASIS::CALIBRATION::CheckerboardDetection&) {});
  OASIS::CheckerboardDetectorNodeTestAccess::SetStatusCallback(detectorNode, [&statuses](bool found)
                                                               { statuses.push_back(found); });

  const sensor_msgs::msg::Image::ConstSharedPtr imageMessage = CreateMono8Image();
  EXPECT_NO_THROW(OASIS::CheckerboardDetectorNodeTestAccess::OnImage(detectorNode, imageMessage));
  EXPECT_NO_THROW(OASIS::CheckerboardDetectorNodeTestAccess::OnImage(detectorNode, imageMessage));

  EXPECT_EQ(detectionCallCount, 2U);
  EXPECT_EQ(statuses, (std::vector<bool>{false, true}));
}

TEST_F(CheckerboardDetectorNodeTest, DebugExceptionDoesNotReplaceDetectionStatus)
{
  rclcpp::Node node{"checkerboard_debug_exception_test"};
  OASIS::CheckerboardDetectorNode detectorNode{node};
  std::vector<bool> statuses;

  OASIS::CheckerboardDetectorNodeTestAccess::SetDetectionCallback(
      detectorNode, [](const cv::Mat&, OASIS::CALIBRATION::CheckerboardDetection& detection)
      { detection.found = true; });
  OASIS::CheckerboardDetectorNodeTestAccess::SetDebugImageCallback(
      detectorNode, [](const sensor_msgs::msg::Image::ConstSharedPtr&, const cv::Mat&,
                       const OASIS::CALIBRATION::CheckerboardDetection&)
      { CV_Error(cv::Error::StsError, "Injected debug image failure"); });
  OASIS::CheckerboardDetectorNodeTestAccess::SetStatusCallback(detectorNode, [&statuses](bool found)
                                                               { statuses.push_back(found); });

  EXPECT_NO_THROW(
      OASIS::CheckerboardDetectorNodeTestAccess::OnImage(detectorNode, CreateMono8Image()));
  EXPECT_EQ(statuses, (std::vector<bool>{true}));
}

TEST_F(CheckerboardDetectorNodeTest, Mono16IsConvertedAndUnsupportedDepthIsRejected)
{
  rclcpp::Node node{"checkerboard_conversion_contract_test"};
  OASIS::CheckerboardDetectorNode detectorNode{node};
  std::vector<bool> statuses;
  size_t detectionCallCount = 0;

  OASIS::CheckerboardDetectorNodeTestAccess::SetDetectionCallback(
      detectorNode,
      [&detectionCallCount](const cv::Mat& gray,
                            OASIS::CALIBRATION::CheckerboardDetection& detection)
      {
        ++detectionCallCount;
        EXPECT_EQ(gray.type(), CV_8UC1);
        EXPECT_EQ(gray.at<uint8_t>(0, 0), 255);
        detection.found = true;
      });
  OASIS::CheckerboardDetectorNodeTestAccess::SetDebugImageCallback(
      detectorNode, [](const sensor_msgs::msg::Image::ConstSharedPtr&, const cv::Mat&,
                       const OASIS::CALIBRATION::CheckerboardDetection&) {});
  OASIS::CheckerboardDetectorNodeTestAccess::SetStatusCallback(detectorNode, [&statuses](bool found)
                                                               { statuses.push_back(found); });

  const cv::Mat unsupportedImage{4, 4, CV_32FC1, cv::Scalar{1.0}};
  EXPECT_NO_THROW(OASIS::CheckerboardDetectorNodeTestAccess::OnImage(
      detectorNode, CreateImage(unsupportedImage, sensor_msgs::image_encodings::TYPE_32FC1)));

  const cv::Mat mono16Image{4, 4, CV_16UC1, cv::Scalar{65535}};
  EXPECT_NO_THROW(OASIS::CheckerboardDetectorNodeTestAccess::OnImage(
      detectorNode, CreateImage(mono16Image, sensor_msgs::image_encodings::MONO16)));

  EXPECT_EQ(detectionCallCount, 1U);
  EXPECT_EQ(statuses, (std::vector<bool>{false, true}));
}
} // namespace
