/*
 *  Copyright (C) 2020-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "VisionGraph.h"

#include "api/ImageProc.h"
#include "api/Video.h"
#include "kernels/cpu/CPUImageProc.h"
#include "kernels/cpu/CPUVideo.h"

#include <tuple>

#include <opencv2/gapi/cpu/imgproc.hpp>
#include <opencv2/gapi/cpu/video.hpp>
#include <opencv2/gapi/gcomputation.hpp>

using namespace OASIS;
using namespace VIDEO;

VisionGraph::~VisionGraph() = default;

void VisionGraph::Compile(unsigned int width,
                          unsigned int height,
                          const cv::Mat& currentFrame,
                          const cv::Mat& currentGrayscale,
                          const cv::Mat& previousGrayscale)
{
  // Inputs
  std::vector<cv::Point2f> currentPoints;
  std::vector<std::vector<cv::Point2f>> pointHistory;
  cv::Mat previousCameraMatrix;
  cv::Scalar maxFeatures;
  cv::Scalar minDistance;

  // Declare graph
  // The version of a pipeline expression with a lambda-based constructor is
  // used to keep all temporary objects in a dedicated scope

  // Convert to grayscale
  cv::GComputation grayscalePipeline(
      []()
      {
        // Input
        cv::GMat rgbaImage;

        // Output
        cv::GMat grayscaleImage;

        grayscaleImage = IMAGE::RGBA2Gray(rgbaImage);

        return cv::GComputation(cv::GIn(rgbaImage), cv::GOut(grayscaleImage));
      });

  // Find features
  cv::GComputation featurePipeline(
      [width, height]()
      {
        // Input
        cv::GMat grayscaleImage;
        cv::GScalar maxFeatures;
        cv::GScalar minDistance;

        // Output
        cv::GArray<cv::Point2f> features;

        features = IMAGE::GoodFeaturesToTrack(grayscaleImage, maxFeatures, minDistance);

        return cv::GComputation(cv::GIn(grayscaleImage, maxFeatures, minDistance),
                                cv::GOut(features));
      });

  // Calculate optical flow
  cv::GComputation opticalFlowPipeline(
      []()
      {
        // Input
        cv::GMat prevImg;
        cv::GMat nextImg;
        cv::GArray<cv::Point2f> previousPoints;
        cv::GArray<std::vector<cv::Point2f>> pointHistory;

        // Intermediate
        cv::GArray<cv::Point2f> predictedPoints;

        // Output
        cv::GArray<cv::Point2f> newPoints;
        cv::GArray<uchar> status;
        cv::GArray<float> errors;

        // Predict next discovered points for optical flow
        predictedPoints = VIDEO::PredictPoints(pointHistory);

        // Perform optical flow calculation
        std::tie(newPoints, status, errors) =
            VIDEO::CalcOpticalFlow(prevImg, nextImg, previousPoints, predictedPoints);

        return cv::GComputation(cv::GIn(prevImg, nextImg, previousPoints, pointHistory),
                                cv::GOut(newPoints, status, errors));
      });

  // Declare custom and gapi kernels
  static auto kernels =
      cv::gapi::combine(cv::gapi::imgproc::cpu::kernels(), cv::gapi::video::cpu::kernels(),
                        IMAGE::kernels(), VIDEO::kernels());

  // Compile computation graphs in serial mode
  m_applyGrayscale =
      grayscalePipeline.compile(cv::descr_of(currentFrame), cv::compile_args(kernels));
  m_findFeatures =
      featurePipeline.compile(cv::descr_of(currentGrayscale), cv::descr_of(maxFeatures),
                              cv::descr_of(minDistance), cv::compile_args(kernels));
  m_calcOpticalFlow = opticalFlowPipeline.compile(
      cv::descr_of(previousGrayscale), cv::descr_of(currentGrayscale), cv::descr_of(currentPoints),
      cv::descr_of(pointHistory), cv::compile_args(kernels));
}

void VisionGraph::ApplyGrayscale(
    // Input
    const cv::Mat& currentFrame,
    // Output
    cv::Mat& currentGrayscale)
{
  auto inVector = cv::gin(currentFrame);
  auto outVector = cv::gout(currentGrayscale);
  m_applyGrayscale(std::move(inVector), std::move(outVector));
}

void VisionGraph::FindFeatures(
    // Input
    const cv::Mat& currentGrayscale,
    unsigned int maxFeatures,
    double minDistance,
    // Output
    std::vector<cv::Point2f>& currentPoints)
{
  cv::Scalar maxFeaturesScalar(maxFeatures > 0 ? static_cast<double>(maxFeatures) : -1.0);
  cv::Scalar minDistanceScalar(minDistance);

  auto inVector = cv::gin(currentGrayscale, maxFeaturesScalar, minDistanceScalar);
  auto outVector = cv::gout(currentPoints);
  m_findFeatures(std::move(inVector), std::move(outVector));
}

void VisionGraph::CalcOpticalFlow(
    // Input
    const cv::Mat& previousGrayscale,
    const cv::Mat& currentGrayscale,
    const std::vector<cv::Point2f>& previousPoints,
    const std::vector<std::vector<cv::Point2f>>& pointHistory,
    // Output
    std::vector<cv::Point2f>& currentPoints,
    std::vector<uchar>& status,
    std::vector<float>& errors)
{
  auto inVector = cv::gin(previousGrayscale, currentGrayscale, previousPoints, pointHistory);
  auto outVector = cv::gout(currentPoints, status, errors);
  m_calcOpticalFlow(std::move(inVector), std::move(outVector));
}
