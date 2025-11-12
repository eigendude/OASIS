/*
 *  Copyright (C) 2021-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "MonocularSlam.h"

#include <sophus/se3.hpp>

using namespace OASIS;
using namespace SLAM;

MonocularSlam::MonocularSlam(rclcpp::Node& node, const std::string& mapImageTopic)
  : MonocularSlamBase(node, mapImageTopic)
{
}

MonocularSlam::~MonocularSlam() = default;

bool MonocularSlam::Initialize(const std::string& vocabularyFile, const std::string& settingsFile)
{
  return InitializeSystem(vocabularyFile, settingsFile, ORB_SLAM3::System::MONOCULAR);
}

void MonocularSlam::Deinitialize()
{
  DeinitializeSystem();
}

Eigen::Isometry3f MonocularSlam::TrackFrame(const cv::Mat& rgbImage, double timestamp)
{
  ORB_SLAM3::System* slam = GetSlam();
  if (slam == nullptr)
    return Eigen::Isometry3f::Identity();

  const Sophus::SE3f sophusPose = slam->TrackMonocular(rgbImage, timestamp);

  Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
  pose.matrix() = sophusPose.matrix();

  return pose;
}
