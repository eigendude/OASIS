/*
 *  Copyright (C) 2020-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "Scene.h"

using namespace OASIS::SCENE;

TplDoubles CalcSceneScore(const cv::GMat& prevImg,
                          const cv::GMat& nextImg,
                          const cv::GOpaque<double>& prevMafd,
                          unsigned int width,
                          unsigned int height)
{
  return GCalcSceneScore::on(prevImg, nextImg, prevMafd, width, height);
}
