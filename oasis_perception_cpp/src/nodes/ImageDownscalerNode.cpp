################################################################################
#
#  Copyright (C) 2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

#include "ImageDownscalerNode.h"

#include "image/ImageDownscaler.h"

#include <cstdint>
#include <exception>
#include <memory>
#include <string>
#include <string_view>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

using namespace OASIS;

namespace
{
constexpr std::string_view IMAGE_TOPIC = "image";

constexpr std::string_view SYSTEM_ID_PARAMETER = "system_id";
constexpr std::string_view DEFAULT_SYSTEM_ID = "";

constexpr std::string_view IMAGE_TRANSPORT_PARAMETER = "image_transport";
constexpr std::string_view DEFAULT_IMAGE_TRANSPORT = "raw";

constexpr std::string_view OUTPUT_SUFFIX_PARAMETER = "output_suffix";
constexpr std::string_view DEFAULT_OUTPUT_SUFFIX = "sd";

constexpr std::string_view MAX_WIDTH_PARAMETER = "max_width";
constexpr int64_t DEFAULT_MAX_WIDTH = 640;

constexpr std::string_view MAX_HEIGHT_PARAMETER = "max_height";
constexpr int64_t DEFAULT_MAX_HEIGHT = 480;
} // namespace

ImageDownscalerNode::ImageDownscalerNode(rclcpp::Node& node) : m_node(node)
{
  m_node.declare_parameter<std::string>(SYSTEM_ID_PARAMETER.data(), DEFAULT_SYSTEM_ID.data());
  m_node.declare_parameter<std::string>(IMAGE_TRANSPORT_PARAMETER.data(),
                                        DEFAULT_IMAGE_TRANSPORT.data());
  m_node.declare_parameter<std::string>(OUTPUT_SUFFIX_PARAMETER.data(),
                                        DEFAULT_OUTPUT_SUFFIX.data());
  m_node.declare_parameter<int64_t>(MAX_WIDTH_PARAMETER.data(), DEFAULT_MAX_WIDTH);
  m_node.declare_parameter<int64_t>(MAX_HEIGHT_PARAMETER.data(), DEFAULT_MAX_HEIGHT);
}

ImageDownscalerNode::~ImageDownscalerNode() = default;

bool ImageDownscalerNode::Initialize()
{
  RCLCPP_INFO(m_node.get_logger(), "Starting image downscaler...");

  std::string systemId;
  if (!m_node.get_parameter(SYSTEM_ID_PARAMETER.data(), systemId))
  {
    RCLCPP_ERROR(m_node.get_logger(), "Missing system ID parameter '%s'", SYSTEM_ID_PARAMETER.data());
    return false;
  }

  if (systemId.empty())
  {
    RCLCPP_ERROR(m_node.get_logger(), "System ID parameter '%s' is empty", SYSTEM_ID_PARAMETER.data());
    return false;
  }

  std::string imageTransport;
  if (!m_node.get_parameter(IMAGE_TRANSPORT_PARAMETER.data(), imageTransport))
  {
    RCLCPP_ERROR(m_node.get_logger(), "Missing image transport parameter '%s'",
                 IMAGE_TRANSPORT_PARAMETER.data());
    return false;
  }

  if (imageTransport.empty())
    imageTransport = std::string{DEFAULT_IMAGE_TRANSPORT};

  std::string outputSuffix;
  if (!m_node.get_parameter(OUTPUT_SUFFIX_PARAMETER.data(), outputSuffix))
  {
    RCLCPP_ERROR(m_node.get_logger(), "Missing output suffix parameter '%s'",
                 OUTPUT_SUFFIX_PARAMETER.data());
    return false;
  }

  if (outputSuffix.empty())
    outputSuffix = std::string{DEFAULT_OUTPUT_SUFFIX};

  int64_t maxWidthParam = DEFAULT_MAX_WIDTH;
  if (!m_node.get_parameter(MAX_WIDTH_PARAMETER.data(), maxWidthParam))
    maxWidthParam = DEFAULT_MAX_WIDTH;

  int64_t maxHeightParam = DEFAULT_MAX_HEIGHT;
  if (!m_node.get_parameter(MAX_HEIGHT_PARAMETER.data(), maxHeightParam))
    maxHeightParam = DEFAULT_MAX_HEIGHT;

  if (maxWidthParam <= 0 || maxHeightParam <= 0)
  {
    RCLCPP_ERROR(m_node.get_logger(),
                 "Invalid maximum dimensions %ldx%ld. Both values must be positive.", maxWidthParam,
                 maxHeightParam);
    return false;
  }

  std::string imageTopic = systemId;
  imageTopic.push_back('_');
  imageTopic.append(IMAGE_TOPIC);

  std::string downscaledTopic = systemId;
  downscaledTopic.push_back('_');
  downscaledTopic.append(outputSuffix);

  RCLCPP_INFO(m_node.get_logger(), "System ID: %s", systemId.c_str());
  RCLCPP_INFO(m_node.get_logger(), "Image topic: %s", imageTopic.c_str());
  RCLCPP_INFO(m_node.get_logger(), "Image transport: %s", imageTransport.c_str());
  RCLCPP_INFO(m_node.get_logger(), "Downscaled topic: %s", downscaledTopic.c_str());
  RCLCPP_INFO(m_node.get_logger(), "Max dimensions: %ldx%ld", maxWidthParam, maxHeightParam);

  try
  {
    auto nodeShared = m_node.shared_from_this();
    m_downscaler = std::make_unique<IMAGE::ImageDownscaler>(
        nodeShared, imageTopic, downscaledTopic, imageTransport, static_cast<int>(maxWidthParam),
        static_cast<int>(maxHeightParam));
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(m_node.get_logger(), "Failed to initialize image downscaler: %s", e.what());
    return false;
  }

  RCLCPP_INFO(m_node.get_logger(), "Started image downscaler");

  return true;
}

void ImageDownscalerNode::Deinitialize()
{
  m_downscaler.reset();
}

