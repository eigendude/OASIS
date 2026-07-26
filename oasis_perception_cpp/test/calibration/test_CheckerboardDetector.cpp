/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "calibration/CheckerboardDetector.h"

#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/imgproc.hpp>

using OASIS::CALIBRATION::CheckerboardDetection;
using OASIS::CALIBRATION::CheckerboardDetector;
using OASIS::CALIBRATION::CheckerboardDetectorOptions;

namespace
{
CheckerboardDetectorOptions CreateDefaultOptions()
{
  CheckerboardDetectorOptions options{};
  options.checkerboardWidth = 8;
  options.checkerboardHeight = 6;
  options.useSbDetector = true;
  options.refineCorners = true;
  options.adaptiveThresh = true;
  options.normalizeImage = true;
  options.fastCheck = true;
  return options;
}

cv::Mat CreateCheckerboardImage(int innerCornersX, int innerCornersY)
{
  constexpr int squareSize = 72;
  constexpr int margin = 72;

  const int squaresX = innerCornersX + 1;
  const int squaresY = innerCornersY + 1;
  cv::Mat image{(squaresY * squareSize) + (2 * margin), (squaresX * squareSize) + (2 * margin),
                CV_8UC1, cv::Scalar{255}};

  for (int y = 0; y < squaresY; ++y)
  {
    for (int x = 0; x < squaresX; ++x)
    {
      if ((x + y) % 2 == 0)
      {
        const cv::Rect square{margin + (x * squareSize), margin + (y * squareSize), squareSize,
                              squareSize};
        image(square).setTo(cv::Scalar{0});
      }
    }
  }

  return image;
}
} // namespace

TEST(CheckerboardDetector, SbDetectorDetectsCheckerboard)
{
  const CheckerboardDetectorOptions options = CreateDefaultOptions();
  CheckerboardDetector detector{options};

  const cv::Mat image =
      CreateCheckerboardImage(options.checkerboardWidth, options.checkerboardHeight);
  const CheckerboardDetection detection = detector.Detect(image);

  EXPECT_TRUE(detection.found);
  EXPECT_EQ(detection.corners.size(),
            static_cast<size_t>(options.checkerboardWidth * options.checkerboardHeight));
}

TEST(CheckerboardDetector, ClassicDetectorDetectsCheckerboard)
{
  CheckerboardDetectorOptions options = CreateDefaultOptions();
  options.useSbDetector = false;
  CheckerboardDetector detector{options};

  const cv::Mat image =
      CreateCheckerboardImage(options.checkerboardWidth, options.checkerboardHeight);
  const CheckerboardDetection detection = detector.Detect(image);

  EXPECT_TRUE(detection.found);
  EXPECT_EQ(detection.corners.size(),
            static_cast<size_t>(options.checkerboardWidth * options.checkerboardHeight));
}

TEST(CheckerboardDetector, RejectsBlankImage)
{
  const CheckerboardDetectorOptions options = CreateDefaultOptions();
  CheckerboardDetector detector{options};

  const cv::Mat image{480, 640, CV_8UC1, cv::Scalar{127}};
  const CheckerboardDetection detection = detector.Detect(image);

  EXPECT_FALSE(detection.found);
  EXPECT_TRUE(detection.corners.empty());
}

TEST(CheckerboardDetector, RejectsEmptyImageWithoutThrowing)
{
  const CheckerboardDetector detector{CreateDefaultOptions()};

  CheckerboardDetection detection{};
  EXPECT_NO_THROW(detection = detector.Detect(cv::Mat{}));
  EXPECT_FALSE(detection.found);
  EXPECT_TRUE(detection.corners.empty());
}

TEST(CheckerboardDetector, TinyImageDoesNotThrow)
{
  const CheckerboardDetector detector{CreateDefaultOptions()};
  const cv::Mat image{1, 1, CV_8UC1, cv::Scalar{0}};

  CheckerboardDetection detection{};
  EXPECT_NO_THROW(detection = detector.Detect(image));
  EXPECT_FALSE(detection.found);
}

TEST(CheckerboardDetector, RejectsUnsupportedImageTypesWithoutThrowing)
{
  const CheckerboardDetector detector{CreateDefaultOptions()};
  const std::vector<cv::Mat> images{
      cv::Mat{480, 640, CV_8UC3, cv::Scalar{0, 0, 0}},
      cv::Mat{480, 640, CV_16UC1, cv::Scalar{0}},
      cv::Mat{480, 640, CV_32FC1, cv::Scalar{0.0}},
  };

  for (const cv::Mat& image : images)
  {
    CheckerboardDetection detection{};
    EXPECT_NO_THROW(detection = detector.Detect(image));
    EXPECT_FALSE(detection.found);
    EXPECT_TRUE(detection.corners.empty());
  }
}

TEST(CheckerboardDetector, InvalidFrameDoesNotPreventLaterDetection)
{
  const CheckerboardDetectorOptions options = CreateDefaultOptions();
  const CheckerboardDetector detector{options};

  const CheckerboardDetection invalidDetection = detector.Detect(cv::Mat{});
  const CheckerboardDetection validDetection = detector.Detect(
      CreateCheckerboardImage(options.checkerboardWidth, options.checkerboardHeight));

  EXPECT_FALSE(invalidDetection.found);
  EXPECT_TRUE(validDetection.found);
}

TEST(CheckerboardDetector, InvalidDimensionsThrow)
{
  CheckerboardDetectorOptions options = CreateDefaultOptions();

  options.checkerboardWidth = 0;
  EXPECT_THROW(CheckerboardDetector{options}, std::invalid_argument);

  options = CreateDefaultOptions();
  options.checkerboardHeight = 0;
  EXPECT_THROW(CheckerboardDetector{options}, std::invalid_argument);
}
