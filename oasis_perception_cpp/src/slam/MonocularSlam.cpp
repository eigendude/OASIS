/*
 *  Copyright (C) 2021-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "MonocularSlam.h"

#include <filesystem>
#include <stdexcept>

#include <System.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/transport_hints.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/image_encodings.hpp>

using namespace OASIS;
using namespace SLAM;

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
      RCLCPP_FATAL(logger, "Monocular SLAM settings file is missing: %s", settingsPath.c_str());
      throw std::runtime_error("Monocular SLAM settings file not found");
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

MonocularSlam::MonocularSlam(rclcpp::Node& node)
  : m_logger(std::make_unique<rclcpp::Logger>(node.get_logger()))
{
}

MonocularSlam::~MonocularSlam() = default;

bool MonocularSlam::Initialize()
{
  const std::string vocabularyFile = GetVocabularyFile(*m_logger);
  const std::string settingsFile = GetSettingsFile(*m_logger);

  RCLCPP_INFO(*m_logger, "Using ORB-SLAM3 vocabulary file: %s", vocabularyFile.c_str());
  RCLCPP_INFO(*m_logger, "Using monocular SLAM settings file: %s", settingsFile.c_str());

  m_slam = std::make_unique<ORB_SLAM3::System>(vocabularyFile, settingsFile,
                                               ORB_SLAM3::System::MONOCULAR, false);

  return true;
}

void MonocularSlam::Deinitialize()
{
  if (m_slam)
  {
    // Stop all threads
    m_slam->Shutdown();

    //m_slam->SaveTrajectoryEuRoC("CameraTrajectory.txt");
    //m_slam->SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");

    m_slam.reset();
  }
}

void MonocularSlam::ReceiveImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(*m_logger, "cv_bridge exception: %s", e.what());
    return;
  }

  double tframe = 0.0; // TODO

  // Pass the image to the SLAM system
  m_slam->TrackMonocular(cv_ptr->image, tframe);
}
