/*
 *  Copyright (C) 2025-2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */
#pragma once

#include "MediaPipeFacade.h"

namespace oasis_perception
{
class PoseLandmarker
{
public:
  PoseLandmarker();
  ~PoseLandmarker();

  bool Initialize(const mediapipe_facade::PoseLandmarkerConfig& config);

  mediapipe_facade::PoseDetectionResult Detect(const mediapipe_facade::PoseDetectionInput& input);

private:
  mediapipe_facade::PoseLandmarker m_facade;
};
} // namespace oasis_perception
