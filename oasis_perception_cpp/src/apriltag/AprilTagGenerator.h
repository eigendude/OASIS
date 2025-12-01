/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>

#include <opencv2/core.hpp>
#include <rclcpp/logger.hpp>

extern "C"
{
#include <apriltag/apriltag.h>
}

namespace OASIS
{

class AprilTagGenerator
{
public:
  explicit AprilTagGenerator(const rclcpp::Logger& logger);
  ~AprilTagGenerator();

  cv::Mat Generate(const std::string& family, int32_t id, int scale = 8);

private:
  using FamilyCreateFn = apriltag_family_t* (*)();
  using FamilyDestroyFn = void (*)(apriltag_family_t*);

  struct FamilyFactory
  {
    FamilyCreateFn create{};
    FamilyDestroyFn destroy{};
  };

  using AprilTagFamilyPtr = std::unique_ptr<apriltag_family_t, FamilyDestroyFn>;

  static std::string NormalizeFamily(const std::string& family);

  static std::optional<FamilyFactory> GetFamilyFactory(const std::string& family);

  apriltag_family_t* GetFamily(const std::string& family);

  rclcpp::Logger m_logger;

  std::unordered_map<std::string, AprilTagFamilyPtr> m_families;
};

} // namespace OASIS
