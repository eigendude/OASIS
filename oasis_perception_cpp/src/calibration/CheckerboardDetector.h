/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <vector>

#include <opencv2/core.hpp>

namespace OASIS
{
namespace CALIBRATION
{
/*!
 * \brief Checkerboard detection options
 */
struct CheckerboardDetectorOptions
{
  /*!
   * \brief Number of inner checkerboard corners along the image x-axis
   *
   * Unit: corners. Expected range: > 0. Used with height to form the OpenCV
   * pattern size passed to checkerboard detectors.
   */
  int checkerboardWidth;

  /*!
   * \brief Number of inner checkerboard corners along the image y-axis
   *
   * Unit: corners. Expected range: > 0. Used with width to form the OpenCV
   * pattern size passed to checkerboard detectors.
   */
  int checkerboardHeight;

  /*!
   * \brief Whether to use OpenCV's sector-based checkerboard detector
   *
   * Expected range: boolean. When true, detection uses
   * cv::findChessboardCornersSB. When false, detection uses the classic
   * cv::findChessboardCorners path.
   */
  bool useSbDetector;

  /*!
   * \brief Whether classic detector corners should be sub-pixel refined
   *
   * Expected range: boolean. Used only after a successful classic detector
   * pass. Ignored by the sector-based detector.
   */
  bool refineCorners;

  /*!
   * \brief Whether classic detection should use adaptive thresholding
   *
   * Expected range: boolean. Maps to cv::CALIB_CB_ADAPTIVE_THRESH for classic
   * checkerboard detection.
   */
  bool adaptiveThresh;

  /*!
   * \brief Whether classic detection should normalize the input image
   *
   * Expected range: boolean. Maps to cv::CALIB_CB_NORMALIZE_IMAGE for classic
   * checkerboard detection.
   */
  bool normalizeImage;

  /*!
   * \brief Whether classic detection should run OpenCV's fast rejection check
   *
   * Expected range: boolean. Maps to cv::CALIB_CB_FAST_CHECK for classic
   * checkerboard detection.
   */
  bool fastCheck;
};

/*!
 * \brief Checkerboard detection result
 */
struct CheckerboardDetection
{
  /*!
   * \brief True when the configured checkerboard pattern was found
   *
   * Expected range: boolean. Computed from OpenCV's checkerboard detector
   * return value for the latest processed image.
   */
  bool found{false};

  /*!
   * \brief Detected inner-corner coordinates in image pixel coordinates
   *
   * Unit: pixels. Expected range: empty when not found, otherwise one point
   * per configured inner corner in OpenCV checkerboard order.
   */
  std::vector<cv::Point2f> corners;
};

class CheckerboardDetector
{
public:
  explicit CheckerboardDetector(CheckerboardDetectorOptions options);

  CheckerboardDetection Detect(const cv::Mat& grayImage) const;

  const CheckerboardDetectorOptions& Options() const { return m_options; }

private:
  CheckerboardDetectorOptions m_options;
};
} // namespace CALIBRATION
} // namespace OASIS
