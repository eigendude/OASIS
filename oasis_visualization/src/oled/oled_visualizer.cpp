/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "oled/oled_visualizer.hpp"

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace OASIS::Visualization
{
namespace
{
constexpr const char* PARAM_FOCAL_LENGTH = "focal_length";
constexpr const char* PARAM_CAMERA_DISTANCE = "camera_distance";
constexpr const char* PARAM_MODEL_SCALE = "model_scale";

void ValidateConfig(const OledVisualizerConfig& config)
{
  if (config.width <= 0 || config.height <= 0)
    throw std::invalid_argument("OLED dimensions must be greater than zero");

  const auto requirePositive = [](const char* name, double value)
  {
    if (!std::isfinite(value) || value <= 0.0)
    {
      throw std::invalid_argument(std::string{name} + " must be finite and greater than zero");
    }
  };
  requirePositive(PARAM_FOCAL_LENGTH, config.focal_length);
  requirePositive(PARAM_CAMERA_DISTANCE, config.camera_distance);
  requirePositive(PARAM_MODEL_SCALE, config.model_scale);

  const double halfWidth = 0.5 * (config.width - 1) * config.model_scale;
  if (halfWidth >= config.camera_distance)
  {
    throw std::invalid_argument("model_scale and camera_distance place the plane at or behind the "
                                "camera during rotation");
  }
}
} // namespace

OledVisualizer::OledVisualizer(const std::string& image_path, OledVisualizerConfig config)
  : config_(config)
{
  ValidateConfig(config_);
  source_ = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
  if (source_.empty())
    throw std::runtime_error("unable to read OLED image: " + image_path);

  if (source_.cols != config_.width || source_.rows != config_.height)
  {
    throw std::runtime_error("OLED image must be exactly " + std::to_string(config_.width) + "x" +
                             std::to_string(config_.height) + " pixels, but is " +
                             std::to_string(source_.cols) + "x" + std::to_string(source_.rows));
  }

  cv::threshold(source_, source_, 127, 255, cv::THRESH_BINARY);
  cv::flip(source_, mirrored_source_, 1);
}

cv::Mat OledVisualizer::Render(double rotation_angle) const
{
  if (!std::isfinite(rotation_angle))
    throw std::invalid_argument("rotation_angle must be finite");

  const double cosine = std::cos(rotation_angle);
  const double sine = std::sin(rotation_angle);

  // model_scale scales the 3D plane before perspective projection
  const double halfWidth = 0.5 * (config_.width - 1) * config_.model_scale;
  const double halfHeight = 0.5 * (config_.height - 1) * config_.model_scale;

  std::vector<cv::Point2f> destination;
  destination.reserve(4);
  for (const cv::Point2d& corner :
       {cv::Point2d(-halfWidth, -halfHeight), cv::Point2d(halfWidth, -halfHeight),
        cv::Point2d(halfWidth, halfHeight), cv::Point2d(-halfWidth, halfHeight)})
  {
    const double rotatedX = corner.x * cosine;
    const double rotatedZ = -corner.x * sine;
    const double depth = config_.camera_distance + rotatedZ;
    destination.emplace_back(
        static_cast<float>(0.5 * config_.width + config_.focal_length * rotatedX / depth),
        static_cast<float>(0.5 * config_.height + config_.focal_length * corner.y / depth));
  }

  const std::vector<cv::Point2f> sourceCorners{
      {0.0F, 0.0F},
      {static_cast<float>(config_.width - 1), 0.0F},
      {static_cast<float>(config_.width - 1), static_cast<float>(config_.height - 1)},
      {0.0F, static_cast<float>(config_.height - 1)}};
  const cv::Mat transform = cv::getPerspectiveTransform(sourceCorners, destination);
  cv::Mat frame;
  const cv::Mat& visibleSource = cosine < 0.0 ? mirrored_source_ : source_;
  cv::warpPerspective(visibleSource, frame, transform, cv::Size(config_.width, config_.height),
                      cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0));
  return frame;
}
} // namespace OASIS::Visualization
