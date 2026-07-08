/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */
#pragma once

#include <cstdint>
#include <string>

namespace oasis_perception::mediapipe_facade
{
struct HelloWorldResult
{
  // True when the MediaPipe hello-world graph or stub completed successfully
  bool success = false;

  // Human-readable status returned by the private backend
  std::string message;

  // Number of output packets observed from the graph
  std::int32_t outputPacketCount = 0;
};

struct PoseLandmarkerConfig
{
  // Stable caller name used by the private backend for diagnostics
  std::string loggingName;
};

struct PoseDetectionStubInput
{
  // Image width in pixels; expected range is [0, sensor maximum]
  std::int32_t width = 0;

  // Image height in pixels; expected range is [0, sensor maximum]
  std::int32_t height = 0;

  // Number of channels per pixel; expected range is [0, 4]
  std::int32_t channelCount = 0;

  // ROS/OpenCV encoding label mapped by the integration layer
  std::string encoding;
};

struct PoseDetectionStubResult
{
  // True when the facade accepted and processed the image metadata
  bool success = false;

  // Human-readable status returned by the private backend
  std::string message;

  // Width * height * channels, used to prove data crossed the facade
  std::int64_t observedScalarCount = 0;
};

HelloWorldResult RunHelloWorld();

class PoseLandmarker
{
public:
  PoseLandmarker();
  ~PoseLandmarker();

  bool Initialize(const PoseLandmarkerConfig& config);
  PoseDetectionStubResult DetectStub(const PoseDetectionStubInput& input);

private:
  bool m_initialized = false;
};
} // namespace oasis_perception::mediapipe_facade
