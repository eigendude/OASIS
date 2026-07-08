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

mediapipe_facade::PoseDetectionResult PoseLandmarker::Detect(
    const mediapipe_facade::PoseDetectionInput& input)
{
  return m_facade.Detect(input);
}

const std::string& PoseLandmarker::LastStatusMessage() const
{
  return m_facade.LastStatusMessage();
}
