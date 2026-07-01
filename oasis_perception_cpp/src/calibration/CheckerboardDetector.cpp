/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "CheckerboardDetector.h"

#include <stdexcept>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace
{
// Maximum cornerSubPix optimizer iterations
constexpr int SUBPIX_MAX_ITERATIONS = 30;

// Required epsilon, in pixels, for cornerSubPix optimizer convergence
constexpr double SUBPIX_EPSILON = 0.001;
} // namespace

namespace OASIS
{
namespace CALIBRATION
{
CheckerboardDetector::CheckerboardDetector(CheckerboardDetectorOptions options) : m_options(options)
{
  if (m_options.checkerboardWidth <= 0)
    throw std::invalid_argument("Checkerboard width must be positive");

  if (m_options.checkerboardHeight <= 0)
    throw std::invalid_argument("Checkerboard height must be positive");
}

CheckerboardDetection CheckerboardDetector::Detect(const cv::Mat& grayImage) const
{
  CheckerboardDetection detection{};
  if (grayImage.empty())
    return detection;

  const cv::Size patternSize{m_options.checkerboardWidth, m_options.checkerboardHeight};

  if (m_options.useSbDetector)
  {
    int flags = 0;
    if (m_options.normalizeImage)
      flags |= cv::CALIB_CB_NORMALIZE_IMAGE;

    detection.found = cv::findChessboardCornersSB(grayImage, patternSize, detection.corners, flags);
    return detection;
  }

  int flags = 0;
  if (m_options.adaptiveThresh)
    flags |= cv::CALIB_CB_ADAPTIVE_THRESH;
  if (m_options.normalizeImage)
    flags |= cv::CALIB_CB_NORMALIZE_IMAGE;
  if (m_options.fastCheck)
    flags |= cv::CALIB_CB_FAST_CHECK;

  detection.found = cv::findChessboardCorners(grayImage, patternSize, detection.corners, flags);
  if (detection.found && m_options.refineCorners)
  {
    // Pixel radius of the cornerSubPix search window
    const cv::Size subpixWindowSize{11, 11};

    // Disabled zone around each cornerSubPix search window center
    const cv::Size subpixZeroZone{-1, -1};

    const cv::TermCriteria criteria{cv::TermCriteria::EPS + cv::TermCriteria::COUNT,
                                    SUBPIX_MAX_ITERATIONS, SUBPIX_EPSILON};
    cv::cornerSubPix(grayImage, detection.corners, subpixWindowSize, subpixZeroZone, criteria);
  }

  return detection;
}
} // namespace CALIBRATION
} // namespace OASIS
