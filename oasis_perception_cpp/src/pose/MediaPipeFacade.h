/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

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

  // Path to a MediaPipe pose landmarker .task model bundle
  std::string modelAssetPath;

  // Maximum poses to detect; expected range is [1, 5]
  std::int32_t maxPoses = 5;
};

struct PoseLandmark
{
  // Normalized x coordinate in image space; expected range is usually [0, 1]
  float x = 0.0F;

  // Normalized y coordinate in image space; expected range is usually [0, 1]
  float y = 0.0F;

  // Model-relative normalized depth; smaller values are closer to camera
  float z = 0.0F;

  // Visibility confidence when supplied by the model; otherwise 0
  float visibility = 0.0F;

  // Presence confidence when supplied by the model; otherwise 0
  float presence = 0.0F;
};

struct PoseDetectionInput
{
  // Image width in pixels; expected range is [1, sensor maximum]
  std::int32_t width = 0;

  // Image height in pixels; expected range is [1, sensor maximum]
  std::int32_t height = 0;

  // Number of channels per pixel; expected range is [3, 4]
  std::int32_t channelCount = 0;

  // Encoding label mapped by the integration layer, e.g. rgb8 or rgba8
  std::string encoding;

  // Pointer to contiguous pixel bytes; valid only for this Detect call
  const std::uint8_t* data = nullptr;

  // Number of bytes reachable from data
  std::size_t dataSize = 0;

  // Source timestamp in milliseconds, computed by the integration layer
  std::int64_t timestampMs = 0;
};

struct PoseDetectionResult
{
  // True when MediaPipe detection completed successfully
  bool success = false;

  // Human-readable status returned by the private backend
  std::string message;

  // Detected poses; each inner vector contains normalized pose landmarks
  std::vector<std::vector<PoseLandmark>> poses;
};

HelloWorldResult RunHelloWorld();

class PoseLandmarker
{
public:
  PoseLandmarker();
  ~PoseLandmarker();

  bool Initialize(const PoseLandmarkerConfig& config);
  PoseDetectionResult Detect(const PoseDetectionInput& input);
  const std::string& LastStatusMessage() const;

  PoseLandmarker(const PoseLandmarker&) = delete;
  PoseLandmarker& operator=(const PoseLandmarker&) = delete;

private:
  void* m_backendHandle = nullptr;
  std::string m_lastStatusMessage;
};
} // namespace oasis_perception::mediapipe_facade
