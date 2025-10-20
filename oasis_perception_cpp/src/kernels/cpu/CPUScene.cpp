/*
 *  Copyright (C) 2020-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "CPUScene.h"

#include "api/Scene.h"
#include "utils/SceneUtils.h"

#include <opencv2/gapi/cpu/gcpukernel.hpp>

namespace OASIS::SCENE
{

// Calculate the scene score given a frame and its previous frame
// clang-format off
GAPI_OCV_KERNEL(GCPUCalcSceneScore, GCalcSceneScore)
{
  static void run(const cv::Mat& prevImg,
                  const cv::Mat& nextImg,
                  double prevMafd,
                  unsigned int width,
                  unsigned int height,
                  double& nextMafd,
                  double& sceneScore)
  {
    nextMafd = UTILS::SceneUtils::CalcSceneMAFD(prevImg.data, nextImg.data, width, height);
    sceneScore = UTILS::SceneUtils::CalcSceneScore(prevMafd, nextMafd);
  }
};
// clang-format on

cv::gapi::GKernelPackage kernels()
{
  static auto pkg = cv::gapi::kernels<GCPUCalcSceneScore>();

  return pkg;
}

} // namespace OASIS::IMAGE
