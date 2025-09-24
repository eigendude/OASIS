/*
 *  Copyright (C) 2020-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "Video.h"

#include <opencv2/video/tracking.hpp>

cv::GArray<cv::Point2f> video::PredictPoints(
    const cv::GArray<std::vector<cv::Point2f>>& pointHistory)
{
  return GPredictPoints::on(pointHistory);
}

cv::gapi::video::GOptFlowLKOutput video::CalcOpticalFlow(const cv::GMat& prevImg,
                                                         const cv::GMat& nextImg,
                                                         const cv::GArray<cv::Point2f>& prevPts,
                                                         const cv::GArray<cv::Point2f>& predPts)
{
  // TODO: Move parameters out of API

  // Window size of optical flow algorithm used to calculate required padding
  // for pyramic levels.
  //
  // Must be no less than winSize argument of calcOpticalFlowPyrLK().
  //const cv::Size winSize = cv::Size(11, 11);
  const cv::Size winSize = cv::Size(21, 21);

  // 0-based maximal pyramid level number.
  //
  // According to Bouguet, 2001, practical values the height of the pyramid
  // (picked heuristically) are 2, 3, 4.
  //
  // If set to 0, pyramids are not used (single level). If set to 1, two
  // levels are used, and so on.
  //
  // The LK algorithm will use as many levels as pyramids, but no more than
  // maxLevel.
  const cv::GScalar& maxLevel = 3;

  // Parameter specifying the termination criteria of the iterative search
  // algorithm.
  //
  // The algorithm terminates after the specified maximum number of
  // iterations or when the search window moves by less than the epsilon.
  const cv::TermCriteria criteria = cv::TermCriteria(
      // The maximum number of iterations or elements to compute
      cv::TermCriteria::COUNT |
          // The desired accuracy or change in parameters at which the iterative
          // algorithm stops
          cv::TermCriteria::EPS,
      // Max number
      30,
      // Epsilon
      0.01);

  const int flags =
      0 |
      // Uses initial estimations, stored in nextPts; if the flag is
      // not set, then prevPts is copied to nextPts and is considered the initial estimate.
      cv::OPTFLOW_USE_INITIAL_FLOW |
      // For the error, use the L1 distance between patches around the original
      // and moved point, divided by number of pixels in a window.
      //
      // Alternatively, set the flag to cv::OPTFLOW_LK_GET_MIN_EIGENVALS to
      // use minimum eigen values as an error measure (see minEigThreshold
      // description).
      //;
      0;

  // The algorithm calculates the minimum eigen value of a 2x2 normal matrix
  // of optical flow equations, divided by number of pixels in a window.
  //
  // If this value is less than minEigThreshold, then a corresponding feature
  // is filtered out and its flow is not processed, so it allows to remove
  // bad points and get a performance boost.
  //
  // The 2x2 normal matrix of optical flow equations is called a spatial
  // gradient matrix in @cite Bouguet00)
  const double minEigThresh = 1e-4;

  return cv::gapi::calcOpticalFlowPyrLK(prevImg, nextImg, prevPts, predPts, winSize, maxLevel,
                                        criteria, flags, minEigThresh);
}
