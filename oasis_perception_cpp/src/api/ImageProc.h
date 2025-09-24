/*
 *  Copyright (C) 2020-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <opencv2/core/types.hpp>
#include <opencv2/gapi/garray.hpp>
#include <opencv2/gapi/gkernel.hpp>
#include <opencv2/gapi/gmat.hpp>

namespace OASIS
{
namespace IMAGE
{
// clang-format off
G_TYPED_KERNEL(GGoodFeatures,
               <cv::GArray<cv::Point2f>(cv::GMat, cv::GScalar, double, cv::GScalar, cv::Mat, int, bool, double)>,
               "com.opticalFlow.imgproc.goodFeaturesToTrack")
{
  static cv::GArrayDesc outMeta(cv::GMatDesc, cv::GScalarDesc, double, cv::GScalarDesc, const cv::Mat&, int, bool, double)
  {
    return cv::empty_array_desc();
  }
};
// clang-format on

/*!
 * \brief Convert RGBA image to grayscale
 *
 * \param rgbaImage The 4-channel RGBA image
 *
 * \return The single-channel grayscale image
 */
cv::GMat RGBA2Gray(const cv::GMat& rgbaImage);

/*!
 * \brief Get some good features to track
 *
 * \param grayscaleImage The single-channel grayscale image
 *
 * \param maxCorners Maximum number of corners to return.
 * If there are more corners than are found, the strongest of them is
 * returned.
 *
 * \param qualityLevel Minimal accepted quality of image corners.
 * This parameter characterizes the minimal accepted quality of
 * corners.
 *
 * The parameter value is multiplied by the best corner quality measure,
 * which is the minimal eigenvalue or the Harris function response.
 *
 * The corners with the quality measure less than the product are rejected.
 *
 * For example, if the best corner has the quality measure = 1500, and the
 * qualityLevel = 0.01, then all the corners with the quality measure less
 * than 15 are rejected.
 *
 * \param minDistance Minimum possible Euclidean distance between the
 * returned corners
 *
 * \return A list of good features to track
 */
cv::GArray<cv::Point2f> GoodFeaturesToTrack(const cv::GMat& grayscaleImage,
                                            const cv::GScalar& maxFeatures,
                                            const cv::GScalar& minDistance,
                                            double qualityLevel = 0.01,
                                            const cv::Mat& mask = cv::Mat(),
                                            int blockSize = 3,
                                            bool useHarrisDetector = false,
                                            double k = 0.04);
} // namespace IMAGE
} // namespace OASIS
