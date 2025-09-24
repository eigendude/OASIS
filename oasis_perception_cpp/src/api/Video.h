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
#include <opencv2/gapi/video.hpp>

namespace OASIS
{
namespace VIDEO
{
// clang-format off
G_TYPED_KERNEL(GPredictPoints,
               <cv::GArray<cv::Point2f>(cv::GArray<std::vector<cv::Point2f>>)>,
               "com.opticalFlow.predictPoints")
{
  static cv::GArrayDesc outMeta(cv::GArrayDesc in)
  {
    return cv::empty_array_desc();
  }
};
// clang-format on

/*!
 * \brief Predict the results of an optical flow calculation given the previous points
 *
 * \param prevPoints The points of the previous frame
 *
 * \return The predicted points in the next frame
 */
cv::GArray<cv::Point2f> PredictPoints(const cv::GArray<std::vector<cv::Point2f>>& prevPoints);

/*!
 * \brief Create a graph node that calucates optical flow
 *
 * @param prevImg First 8-bit input image
 * @param nextImg Second input image of the same size and the same type as prevImg
 * @param prevPts Vector of 2D points for which the flow needs to be found
 * @param predPts Points containing the predicted new positions of input features in the second image
 *
 * @note When OPTFLOW_USE_INITIAL_FLOW flag is passed, the prediction vector
 * must have the same size as in the input
 *
 * @return G-API optical flow output
 */
cv::gapi::video::GOptFlowLKOutput CalcOpticalFlow(const cv::GMat& prevImg,
                                                  const cv::GMat& nextImg,
                                                  const cv::GArray<cv::Point2f>& prevPts,
                                                  const cv::GArray<cv::Point2f>& predPts);
} // namespace VIDEO
} // namespace OASIS
