/*
 *  Copyright (C) 2022-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "MonocularInertialSlam.h"

#include <filesystem>
#include <functional>
#include <stdexcept>

#include <System.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/transport_hints.hpp>
#include <oasis_msgs/msg/i2_c_imu.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/header.hpp>

using namespace OASIS;
using namespace SLAM;
using std::placeholders::_1;

namespace
{
std::string GetVocabularyFile(const rclcpp::Logger& logger)
{
  try
  {
    const std::string shareDir = ament_index_cpp::get_package_share_directory("orb_slam3");
    const auto vocabularyPath = std::filesystem::path(shareDir) / "Vocabulary" / "ORBvoc.txt";

    if (!std::filesystem::exists(vocabularyPath))
    {
      RCLCPP_FATAL(logger, "ORB-SLAM3 vocabulary file is missing: %s", vocabularyPath.c_str());
      throw std::runtime_error("ORB-SLAM3 vocabulary file not found");
    }

    return vocabularyPath.string();
  }
  catch (const std::exception& ex)
  {
    RCLCPP_FATAL(logger, "Failed to locate the 'orb_slam3' package: %s", ex.what());
    throw;
  }
}

std::string GetSettingsFile(const rclcpp::Logger& logger)
{
  try
  {
    const std::string shareDir =
        ament_index_cpp::get_package_share_directory("oasis_perception_cpp");
    const auto settingsPath = std::filesystem::path(shareDir) / "config" / "Webcam.yaml";

    if (!std::filesystem::exists(settingsPath))
    {
      RCLCPP_FATAL(logger, "Monocular inertial SLAM settings file is missing: %s",
                   settingsPath.c_str());
      throw std::runtime_error("Monocular inertial SLAM settings file not found");
    }

    return settingsPath.string();
  }
  catch (const std::exception& ex)
  {
    RCLCPP_FATAL(logger, "Failed to locate the 'oasis_perception_cpp' package: %s", ex.what());
    throw;
  }
}
} // namespace

MonocularInertialSlam::MonocularInertialSlam(std::shared_ptr<rclcpp::Node> node,
                                             const std::string& imageTopic,
                                             const std::string& imuTopic)
  : m_logger(node->get_logger()),
    m_imgTransport(std::make_unique<image_transport::ImageTransport>(node)),
    m_imgSubscriber(std::make_unique<image_transport::Subscriber>()),
    m_slam(std::make_unique<ORB_SLAM3::System>(GetVocabularyFile(m_logger),
                                               GetSettingsFile(m_logger),
                                               ORB_SLAM3::System::IMU_MONOCULAR,
                                               false))
{
  RCLCPP_INFO(m_logger, "Image topic: %s", imageTopic.c_str());
  RCLCPP_INFO(m_logger, "IMU topic: %s", imuTopic.c_str());

  const rclcpp::QoS qos{10};

  auto transportHints = image_transport::TransportHints(node.get(), "compressed");

  *m_imgSubscriber = m_imgTransport->subscribe(imageTopic, 1, &MonocularInertialSlam::ReceiveImage,
                                               this, &transportHints);
  m_imuSubscriber = node->create_subscription<oasis_msgs::msg::I2CImu>(
      imuTopic, qos, std::bind(&MonocularInertialSlam::ImuCallback, this, _1));

  RCLCPP_INFO(m_logger, "Started monocular inertial SLAM");
}

MonocularInertialSlam::~MonocularInertialSlam()
{
  // Stop all threads
  m_slam->Shutdown();

  //m_slam->SaveTrajectoryEuRoC("CameraTrajectory.txt");
  //m_slam->SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
}

void MonocularInertialSlam::ReceiveImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  const std_msgs::msg::Header& header = msg->header;
  const double timestamp =
      static_cast<double>(header.stamp.sec) + static_cast<double>(header.stamp.nanosec) * 1E9;

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

  // Pass the image to the SLAM system
  m_slam->TrackMonocular(cv_ptr->image, timestamp, m_imuMeasurements);

  // TODO: Better IMU synchronization
  m_imuMeasurements.clear();
}

void MonocularInertialSlam::ImuCallback(const oasis_msgs::msg::I2CImu::ConstSharedPtr& msg)
{
  const sensor_msgs::msg::Imu& imuMsg = msg->imu;

  const std_msgs::msg::Header& header = imuMsg.header;
  const double timestamp =
      static_cast<double>(header.stamp.sec) + static_cast<double>(header.stamp.nanosec) * 1E9;

  const geometry_msgs::msg::Vector3& angularVelocity = imuMsg.angular_velocity;
  const geometry_msgs::msg::Vector3& linearAceleration = imuMsg.linear_acceleration;

  const double ax = linearAceleration.x;
  const double ay = linearAceleration.y;
  const double az = linearAceleration.z;

  const double gx = angularVelocity.x;
  const double gy = angularVelocity.y;
  const double gz = angularVelocity.z;

  const cv::Point3f acc(ax, ay, az);
  const cv::Point3f gyr(gx, gy, gz);

  m_imuMeasurements.push_back(ORB_SLAM3::IMU::Point(acc, gyr, timestamp));
}
