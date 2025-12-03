/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "apriltag/AprilTagGenerator.h"

#include <algorithm>
#include <cctype>
#include <string>

#include <opencv2/imgproc.hpp>
#include <rclcpp/logging.hpp>

extern "C"
{
#include <apriltag/apriltag.h>
#include <apriltag/tag16h5.h>
#include <apriltag/tag25h9.h>
#include <apriltag/tag36h10.h>
#include <apriltag/tag36h11.h>
#include <apriltag/tagCircle21h7.h>
#include <apriltag/tagCircle49h12.h>
#include <apriltag/tagCustom48h12.h>
#include <apriltag/tagStandard41h12.h>
#include <apriltag/tagStandard52h13.h>
}

using namespace OASIS;

namespace
{
constexpr int DEFAULT_SCALE = 8;
}

AprilTagGenerator::AprilTagGenerator(const rclcpp::Logger& logger) : m_logger(logger)
{
}

AprilTagGenerator::~AprilTagGenerator() = default;

cv::Mat AprilTagGenerator::Generate(const std::string& family, int32_t id, int scale)
{
  scale = scale > 0 ? scale : DEFAULT_SCALE;

  apriltag_family_t* apriltagFamily = GetFamily(family);
  if (apriltagFamily == nullptr)
    return {};

  if (id < 0 || static_cast<uint32_t>(id) >= apriltagFamily->ncodes)
  {
    RCLCPP_WARN(m_logger, "AprilTag ID %d is out of range for family %s (max id: %d)", id,
                family.c_str(), apriltagFamily->ncodes - 1);
    return {};
  }

  image_u8_t* tagImage = apriltag_to_image(apriltagFamily, id);
  if (tagImage == nullptr)
  {
    RCLCPP_ERROR(m_logger, "Failed to generate AprilTag %s:%d", family.c_str(), id);
    return {};
  }

  cv::Mat cvTag(tagImage->height, tagImage->width, CV_8UC1, tagImage->buf, tagImage->stride);
  cv::Mat tagClone = cvTag.clone();
  image_u8_destroy(tagImage);

  if (scale != 1)
    cv::resize(tagClone, tagClone, cv::Size(), scale, scale, cv::INTER_LINEAR);

  return tagClone;
}

std::string AprilTagGenerator::NormalizeFamily(const std::string& family)
{
  std::string familyLower = family;
  std::transform(familyLower.begin(), familyLower.end(), familyLower.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return familyLower;
}

std::optional<AprilTagGenerator::FamilyFactory> AprilTagGenerator::GetFamilyFactory(
    const std::string& family)
{
  const std::string familyLower = NormalizeFamily(family);

  if (familyLower == "tag16h5")
    return FamilyFactory{tag16h5_create, tag16h5_destroy};
  if (familyLower == "tag25h9")
    return FamilyFactory{tag25h9_create, tag25h9_destroy};
  if (familyLower == "tag36h10")
    return FamilyFactory{tag36h10_create, tag36h10_destroy};
  if (familyLower == "tag36h11")
    return FamilyFactory{tag36h11_create, tag36h11_destroy};
  if (familyLower == "tagcircle21h7")
    return FamilyFactory{tagCircle21h7_create, tagCircle21h7_destroy};
  if (familyLower == "tagcircle49h12")
    return FamilyFactory{tagCircle49h12_create, tagCircle49h12_destroy};
  if (familyLower == "tagstandard41h12")
    return FamilyFactory{tagStandard41h12_create, tagStandard41h12_destroy};
  if (familyLower == "tagstandard52h13")
    return FamilyFactory{tagStandard52h13_create, tagStandard52h13_destroy};
  if (familyLower == "tagcustom48h12")
    return FamilyFactory{tagCustom48h12_create, tagCustom48h12_destroy};

  return std::nullopt;
}

apriltag_family_t* AprilTagGenerator::GetFamily(const std::string& family)
{
  const std::string familyLower = NormalizeFamily(family);

  const auto iter = m_families.find(familyLower);
  if (iter != m_families.end())
    return iter->second.get();

  const auto factory = GetFamilyFactory(familyLower);
  if (!factory)
  {
    RCLCPP_WARN(m_logger, "Unknown AprilTag family: %s", family.c_str());
    return nullptr;
  }

  AprilTagFamilyPtr familyHandle(factory->create(), factory->destroy);
  if (!familyHandle)
  {
    RCLCPP_ERROR(m_logger, "Failed to create AprilTag family: %s", family.c_str());
    return nullptr;
  }

  apriltag_family_t* familyPtr = familyHandle.get();
  m_families.emplace(familyLower, std::move(familyHandle));

  return familyPtr;
}
