/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "oled/oled_visualizer.hpp"

#include <atomic>
#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

using OASIS::Visualization::OledVisualizer;
using OASIS::Visualization::OledVisualizerConfig;

namespace
{
constexpr double PI = 3.14159265358979323846;

constexpr OledVisualizerConfig CONFIG{128, 32, 95.0, 150.0, 1.4};

class TemporaryImage
{
public:
  explicit TemporaryImage(const cv::Mat& image)
  {
    static std::atomic<unsigned int> sequence{0};
    path_ = std::filesystem::temp_directory_path() /
            ("oasis_oled_visualizer_" + std::to_string(sequence.fetch_add(1)) + ".png");
    if (!cv::imwrite(path_.string(), image))
      throw std::runtime_error("failed to create temporary test image");
  }

  ~TemporaryImage()
  {
    std::error_code error;
    std::filesystem::remove(path_, error);
  }

  std::string Path() const { return path_.string(); }

private:
  std::filesystem::path path_;
};

cv::Mat MakePattern()
{
  cv::Mat image = cv::Mat::zeros(32, 128, CV_8UC1);
  cv::rectangle(image, cv::Rect(28, 7, 18, 18), cv::Scalar(173), cv::FILLED);
  return image;
}

cv::Rect NonzeroBounds(const cv::Mat& image)
{
  std::vector<cv::Point> points;
  cv::findNonZero(image, points);
  return cv::boundingRect(points);
}

TEST(OledVisualizerTest, RejectsIncorrectSourceDimensions)
{
  const TemporaryImage image(cv::Mat::zeros(31, 128, CV_8UC1));
  EXPECT_THROW(OledVisualizer(image.Path(), CONFIG), std::runtime_error);
}

TEST(OledVisualizerTest, ProducesMonochromeOutputWithExpectedDimensions)
{
  const TemporaryImage image(MakePattern());
  const OledVisualizer visualizer(image.Path(), CONFIG);
  const cv::Mat output = visualizer.Render(0.0);

  EXPECT_EQ(output.cols, 128);
  EXPECT_EQ(output.rows, 32);
  EXPECT_EQ(output.type(), CV_8UC1);

  cv::Mat invalidPixels;
  cv::inRange(output, cv::Scalar(1), cv::Scalar(254), invalidPixels);
  EXPECT_EQ(cv::countNonZero(invalidPixels), 0);
}

TEST(OledVisualizerTest, UnitScaleProjectsUnscaledModelDimensions)
{
  const TemporaryImage image(cv::Mat(32, 128, CV_8UC1, cv::Scalar(255)));
  constexpr OledVisualizerConfig unitScaleConfig{128, 32, 95.0, 150.0, 1.0};
  const OledVisualizer visualizer(image.Path(), unitScaleConfig);

  // A 127x31 model projects to approximately 80x20 pixels at this distance
  const cv::Rect bounds = NonzeroBounds(visualizer.Render(0.0));
  EXPECT_NEAR(bounds.width, 127.0 * 95.0 / 150.0, 2.0);
  EXPECT_NEAR(bounds.height, 31.0 * 95.0 / 150.0, 2.0);
}

TEST(OledVisualizerTest, ConfiguredScaleMatchesPythonFaceOnProjection)
{
  const TemporaryImage image(cv::Mat(32, 128, CV_8UC1, cv::Scalar(255)));
  const OledVisualizer visualizer(image.Path(), CONFIG);

  const cv::Rect bounds = NonzeroBounds(visualizer.Render(0.0));
  EXPECT_NEAR(bounds.width, 127.0 * 1.4 * 95.0 / 150.0, 2.0);
  EXPECT_NEAR(bounds.height, 31.0 * 1.4 * 95.0 / 150.0, 2.0);
}

TEST(OledVisualizerTest, ConfiguredScaleIsSmallerThanOldCppInterpretation)
{
  const TemporaryImage image(cv::Mat(32, 128, CV_8UC1, cv::Scalar(255)));
  const OledVisualizer visualizer(image.Path(), CONFIG);

  // This recreates the old C++ model width after pixels-to-model conversion
  constexpr double oldModelScale = 128.0 * 1.4 * 150.0 / (95.0 * 127.0);
  constexpr OledVisualizerConfig oldCppConfig{128, 32, 95.0, 150.0, oldModelScale};
  const OledVisualizer oldCppVisualizer(image.Path(), oldCppConfig);

  EXPECT_LT(cv::countNonZero(visualizer.Render(0.0)),
            cv::countNonZero(oldCppVisualizer.Render(0.0)));
}

TEST(OledVisualizerTest, QuarterTurnIsNearlyEdgeOn)
{
  const TemporaryImage image(MakePattern());
  const OledVisualizer visualizer(image.Path(), CONFIG);
  const int facePixels = cv::countNonZero(visualizer.Render(0.0));
  const int edgePixels = cv::countNonZero(visualizer.Render(0.5 * PI));

  EXPECT_GT(facePixels, 0);
  EXPECT_LT(edgePixels, facePixels / 4);
}

TEST(OledVisualizerTest, BackFaceUsesMirroredSource)
{
  const TemporaryImage image(MakePattern());
  const OledVisualizer visualizer(image.Path(), CONFIG);
  const cv::Mat front = visualizer.Render(0.0);
  const cv::Mat back = visualizer.Render(PI);

  EXPECT_EQ(cv::norm(front, back, cv::NORM_INF), 0.0);
}

TEST(OledVisualizerTest, RejectsPlaneAtOrBehindCameraDuringRotation)
{
  const TemporaryImage image(MakePattern());
  constexpr double cameraTouchingScale = 2.0 * 150.0 / 127.0;
  constexpr OledVisualizerConfig touchingConfig{128, 32, 95.0, 150.0, cameraTouchingScale};
  constexpr OledVisualizerConfig behindConfig{128, 32, 95.0, 150.0, cameraTouchingScale + 0.01};

  EXPECT_THROW(OledVisualizer(image.Path(), touchingConfig), std::invalid_argument);
  EXPECT_THROW(OledVisualizer(image.Path(), behindConfig), std::invalid_argument);
}

TEST(OledVisualizerTest, PythonScaleKeepsSourceEdgesVisible)
{
  cv::Mat source = cv::Mat::zeros(32, 128, CV_8UC1);
  cv::rectangle(source, cv::Rect(0, 0, 128, 32), cv::Scalar(255), 1);
  const TemporaryImage image(source);
  const OledVisualizer visualizer(image.Path(), CONFIG);

  EXPECT_GT(cv::countNonZero(visualizer.Render(0.0)), 0);
}
} // namespace
