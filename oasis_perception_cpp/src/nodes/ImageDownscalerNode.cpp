/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "ImageDownscalerNode.h"

#include "image/ImageDownscaler.h"

#include <cstdint>
#include <exception>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

using namespace OASIS;

namespace
{
constexpr std::string_view IMAGE_TOPIC = "image";
constexpr std::string_view CAMERA_INFO_TOPIC = "camera_info";

constexpr std::string_view SYSTEM_ID_PARAMETER = "system_id";
constexpr std::string_view DEFAULT_SYSTEM_ID = "";

constexpr std::string_view IMAGE_TRANSPORT_PARAMETER = "image_transport";
constexpr std::string_view DEFAULT_IMAGE_TRANSPORT = "raw";

constexpr std::string_view OUTPUT_RESOLUTION_PARAMETER = "output_resolution";
constexpr std::string_view DEFAULT_OUTPUT_RESOLUTION = "sd";

constexpr std::string_view OUTPUT_WIDTH_PARAMETER = "output_width";
constexpr int64_t DEFAULT_OUTPUT_WIDTH = -1;

constexpr std::string_view OUTPUT_HEIGHT_PARAMETER = "output_height";
constexpr int64_t DEFAULT_OUTPUT_HEIGHT = -1;

constexpr std::string_view MAX_WIDTH_PARAMETER = "max_width";
constexpr int64_t DEFAULT_MAX_WIDTH = -1;

constexpr std::string_view MAX_HEIGHT_PARAMETER = "max_height";
constexpr int64_t DEFAULT_MAX_HEIGHT = -1;
} // namespace

ImageDownscalerNode::ImageDownscalerNode(rclcpp::Node& node) : m_node(node)
{
  m_node.declare_parameter<std::string>(SYSTEM_ID_PARAMETER.data(), DEFAULT_SYSTEM_ID.data());
  m_node.declare_parameter<std::string>(IMAGE_TRANSPORT_PARAMETER.data(),
                                        DEFAULT_IMAGE_TRANSPORT.data());
  m_node.declare_parameter<std::string>(OUTPUT_RESOLUTION_PARAMETER.data(),
                                        DEFAULT_OUTPUT_RESOLUTION.data());
  m_node.declare_parameter<int64_t>(OUTPUT_WIDTH_PARAMETER.data(), DEFAULT_OUTPUT_WIDTH);
  m_node.declare_parameter<int64_t>(OUTPUT_HEIGHT_PARAMETER.data(), DEFAULT_OUTPUT_HEIGHT);
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
    RCLCPP_ERROR(m_node.get_logger(), "Missing system ID parameter '%s'",
                 SYSTEM_ID_PARAMETER.data());
    return false;
  }

  if (systemId.empty())
  {
    RCLCPP_ERROR(m_node.get_logger(), "System ID parameter '%s' is empty",
                 SYSTEM_ID_PARAMETER.data());
    return false;
  }

  const image_transport::TransportHints transportHints{
      &m_node, std::string{DEFAULT_IMAGE_TRANSPORT}, std::string{IMAGE_TRANSPORT_PARAMETER}};
  const std::string imageTransport = transportHints.getTransport();

  std::string outputResolution;
  if (!m_node.get_parameter(OUTPUT_RESOLUTION_PARAMETER.data(), outputResolution))
  {
    RCLCPP_ERROR(m_node.get_logger(), "Missing output resolution parameter '%s'",
                 OUTPUT_RESOLUTION_PARAMETER.data());
    return false;
  }

  if (outputResolution.empty())
    outputResolution = std::string{DEFAULT_OUTPUT_RESOLUTION};

  int64_t outputWidthParam = DEFAULT_OUTPUT_WIDTH;
  if (!m_node.get_parameter(OUTPUT_WIDTH_PARAMETER.data(), outputWidthParam))
    outputWidthParam = DEFAULT_OUTPUT_WIDTH;

  int64_t outputHeightParam = DEFAULT_OUTPUT_HEIGHT;
  if (!m_node.get_parameter(OUTPUT_HEIGHT_PARAMETER.data(), outputHeightParam))
    outputHeightParam = DEFAULT_OUTPUT_HEIGHT;

  int64_t maxWidthParam = DEFAULT_MAX_WIDTH;
  if (!m_node.get_parameter(MAX_WIDTH_PARAMETER.data(), maxWidthParam))
    maxWidthParam = DEFAULT_MAX_WIDTH;

  int64_t maxHeightParam = DEFAULT_MAX_HEIGHT;
  if (!m_node.get_parameter(MAX_HEIGHT_PARAMETER.data(), maxHeightParam))
    maxHeightParam = DEFAULT_MAX_HEIGHT;

  auto toOptionalDimension = [this](std::string_view parameterName, int64_t rawValue,
                                    std::optional<unsigned int>& destination) -> bool
  {
    if (rawValue <= 0)
    {
      destination.reset();
      return true;
    }

    if (rawValue > std::numeric_limits<unsigned int>::max())
    {
      RCLCPP_ERROR(m_node.get_logger(), "Parameter '%s' value %ld exceeds supported range",
                   parameterName.data(), rawValue);
      return false;
    }

    destination = static_cast<unsigned int>(rawValue);
    return true;
  };

  std::optional<unsigned int> outputWidth;
  if (!toOptionalDimension(OUTPUT_WIDTH_PARAMETER, outputWidthParam, outputWidth))
    return false;

  std::optional<unsigned int> outputHeight;
  if (!toOptionalDimension(OUTPUT_HEIGHT_PARAMETER, outputHeightParam, outputHeight))
    return false;

  std::optional<unsigned int> maxWidth;
  if (!toOptionalDimension(MAX_WIDTH_PARAMETER, maxWidthParam, maxWidth))
    return false;

  std::optional<unsigned int> maxHeight;
  if (!toOptionalDimension(MAX_HEIGHT_PARAMETER, maxHeightParam, maxHeight))
    return false;

  if (!outputWidth && !outputHeight && !maxWidth && !maxHeight)
  {
    RCLCPP_WARN(
        m_node.get_logger(),
        "No size parameters provided for image downscaler. Images will be republished as-is.");
  }

  std::string imageTopic = systemId;
  imageTopic.push_back('_');
  imageTopic.append(IMAGE_TOPIC);

  std::string downscaledTopic = imageTopic;
  downscaledTopic.push_back('_');
  downscaledTopic.append(outputResolution);

  std::string cameraInfoTopic = systemId;
  cameraInfoTopic.push_back('_');
  cameraInfoTopic.append(CAMERA_INFO_TOPIC);

  std::string downscaledCameraInfoTopic = cameraInfoTopic;
  downscaledCameraInfoTopic.push_back('_');
  downscaledCameraInfoTopic.append(outputResolution);

  RCLCPP_INFO(m_node.get_logger(), "System ID: %s", systemId.c_str());
  RCLCPP_INFO(m_node.get_logger(), "Image topic: %s", imageTopic.c_str());
  RCLCPP_INFO(m_node.get_logger(), "Image transport: %s", imageTransport.c_str());
  RCLCPP_INFO(m_node.get_logger(), "Output resolution: %s", outputResolution.c_str());
  RCLCPP_INFO(m_node.get_logger(), "Downscaled topic: %s", downscaledTopic.c_str());
  RCLCPP_INFO(m_node.get_logger(), "Camera info topic: %s", cameraInfoTopic.c_str());
  RCLCPP_INFO(m_node.get_logger(), "Downscaled camera info topic: %s",
              downscaledCameraInfoTopic.c_str());
  if (outputWidth || outputHeight)
  {
    const auto widthLog = outputWidth ? std::to_string(*outputWidth) : std::string{"native"};
    const auto heightLog = outputHeight ? std::to_string(*outputHeight) : std::string{"native"};
    RCLCPP_INFO(m_node.get_logger(), "Output dimensions: %s x %s", widthLog.c_str(),
                heightLog.c_str());
  }
  if (maxWidth || maxHeight)
  {
    const auto widthLog = maxWidth ? std::to_string(*maxWidth) : std::string{"unbounded"};
    const auto heightLog = maxHeight ? std::to_string(*maxHeight) : std::string{"unbounded"};
    RCLCPP_INFO(m_node.get_logger(), "Max dimensions: %s x %s", widthLog.c_str(),
                heightLog.c_str());
  }

  try
  {
    std::shared_ptr<rclcpp::Node> nodeShared = m_node.shared_from_this();
    m_downscaler = std::make_unique<IMAGE::ImageDownscaler>(
        std::move(nodeShared), std::move(imageTopic), std::move(downscaledTopic),
        std::move(imageTransport), std::move(outputWidth), std::move(outputHeight),
        std::move(maxWidth), std::move(maxHeight));
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
