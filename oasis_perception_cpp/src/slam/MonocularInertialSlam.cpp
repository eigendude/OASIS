/*
 *  Copyright (C) 2022-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "MonocularInertialSlam.h"

#include "ros/RosUtils.h"

#include <Eigen/Geometry>
#include <System.h>
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <oasis_msgs/msg/i2_c_imu.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/header.hpp>

using namespace OASIS;
using namespace SLAM;

MonocularInertialSlam::MonocularInertialSlam(rclcpp::Node& node, const std::string& mapImageTopic)
  : m_logger(std::make_unique<rclcpp::Logger>(node.get_logger()))
{
}

MonocularInertialSlam::~MonocularInertialSlam() = default;

bool MonocularInertialSlam::Initialize(const std::string& vocabularyFile,
                                       const std::string& settingsFile)
{
  if (vocabularyFile.empty() || settingsFile.empty())
    return false;

  m_slam = std::make_unique<ORB_SLAM3::System>(vocabularyFile, settingsFile,
                                               ORB_SLAM3::System::IMU_MONOCULAR, false);

  m_imuMeasurements.clear();

  return true;
}

void MonocularInertialSlam::Deinitialize()
{
  if (m_slam)
  {
    // Stop all threads
    m_slam->Shutdown();

    //m_slam->SaveTrajectoryEuRoC("CameraTrajectory.txt");
    //m_slam->SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");

    m_slam.reset();
  }

  m_imuMeasurements.clear();
}

void MonocularInertialSlam::ReceiveImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  if (!m_slam)
    return;

  const std_msgs::msg::Header& header = msg->header;
  const double timestamp = ROS::RosUtils::HeaderStampToSeconds(header);

  cv_bridge::CvImageConstPtr inputImage;
  try
  {
    inputImage = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(*m_logger, "cv_bridge exception: %s", e.what());
    return;
  }

  const cv::Mat& rgbImage = inputImage->image;

  // Pass the image to the SLAM system
  const Sophus::SE3f cameraPose = m_slam->TrackMonocular(rgbImage, timestamp, m_imuMeasurements);

  // TODO: Log status and publish map image

  // TODO: Better IMU synchronization
  m_imuMeasurements.clear();
}

void MonocularInertialSlam::ImuCallback(const oasis_msgs::msg::I2CImu::ConstSharedPtr& msg)
{
  if (!m_slam)
    return;

  const sensor_msgs::msg::Imu& imuMsg = msg->imu;

  const std_msgs::msg::Header& header = imuMsg.header;
  const double timestamp = ROS::RosUtils::HeaderStampToSeconds(header);

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
