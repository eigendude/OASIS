/*
 *  Copyright (C) 2021-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "MonocularSlamBase.h"

namespace OASIS
{
namespace SLAM
{

class MonocularSlam : public MonocularSlamBase
{
public:
  MonocularSlam(rclcpp::Node& node, const std::string& mapImageTopic);
  ~MonocularSlam() override;

  bool Initialize(const std::string& vocabularyFile, const std::string& settingsFile);
  void Deinitialize();

protected:
  Sophus::SE3f TrackFrame(const cv::Mat& rgbImage, double timestamp) override;
};

} // namespace SLAM
} // namespace OASIS
