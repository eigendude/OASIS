/*
 *  Copyright (C) 2025-2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "PoseLandmarker.h"

using namespace oasis_perception;

PoseLandmarker::PoseLandmarker() = default;

PoseLandmarker::~PoseLandmarker() = default;

bool PoseLandmarker::Initialize(const mediapipe_facade::PoseLandmarkerConfig& config)
{
  return m_facade.Initialize(config);
}

mediapipe_facade::PoseDetectionStubResult PoseLandmarker::DetectStub(
    const mediapipe_facade::PoseDetectionStubInput& input)
{
  return m_facade.DetectStub(input);
}
