/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "MediaPipeFacade.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstring>
#include <utility>
#include <vector>

#ifndef OASIS_MEDIAPIPE_BACKEND_AVAILABLE
#error "OASIS_MEDIAPIPE_BACKEND_AVAILABLE must be defined for MediaPipeFacade.cpp"
#endif

namespace oasis_perception::mediapipe_facade
{
namespace
{
constexpr int OASIS_MEDIAPIPE_MAX_POSES = 5;
constexpr int OASIS_MEDIAPIPE_POSE_LANDMARK_COUNT = 33;
constexpr std::size_t STATUS_BUFFER_SIZE = 512;

struct OasisPoseLandmark
{
  float x;
  float y;
  float z;
  float visibility;
  float presence;
};

struct OasisPose
{
  int landmarkCount;
  OasisPoseLandmark landmarks[OASIS_MEDIAPIPE_POSE_LANDMARK_COUNT];
};

struct OasisPoseLandmarkerOutput
{
  int poseCount;
  OasisPose poses[OASIS_MEDIAPIPE_MAX_POSES];
};
} // namespace

extern "C"
{
  int oasis_mediapipe_backend_run_hello_world(char* statusMessage,
                                              std::size_t statusMessageSize,
                                              int* outputPacketCount);

  int oasis_mediapipe_backend_create_pose_landmarker(const char* modelAssetPath,
                                                     int maxPoses,
                                                     char* statusMessage,
                                                     std::size_t statusMessageSize,
                                                     void** handle);

  int oasis_mediapipe_backend_destroy_pose_landmarker(void* handle);

  int oasis_mediapipe_backend_detect_pose(void* handle,
                                          const unsigned char* imageData,
                                          std::size_t imageDataSize,
                                          int width,
                                          int height,
                                          int channelCount,
                                          const char* encoding,
                                          long long timestampMs,
                                          OasisPoseLandmarkerOutput* output,
                                          char* statusMessage,
                                          std::size_t statusMessageSize);
}

namespace
{
std::string GetMessage(const std::array<char, STATUS_BUFFER_SIZE>& buffer)
{
  return std::string{buffer.data()};
}

void CopyStatus(std::array<char, STATUS_BUFFER_SIZE>& buffer, const std::string& message)
{
  const std::size_t copySize = std::min(buffer.size() - 1, message.size());
  std::memcpy(buffer.data(), message.data(), copySize);
  buffer[copySize] = '\0';
}
} // namespace

HelloWorldResult RunHelloWorld()
{
  std::array<char, STATUS_BUFFER_SIZE> statusMessage{};
  int outputPacketCount = 0;

  const bool success = oasis_mediapipe_backend_run_hello_world(
                           statusMessage.data(), statusMessage.size(), &outputPacketCount) == 0;

  return {
      success,
      GetMessage(statusMessage),
      outputPacketCount,
  };
}

PoseLandmarker::PoseLandmarker() = default;

PoseLandmarker::~PoseLandmarker()
{
  if (m_backendHandle != nullptr)
  {
    oasis_mediapipe_backend_destroy_pose_landmarker(m_backendHandle);
    m_backendHandle = nullptr;
  }
}

bool PoseLandmarker::Initialize(const PoseLandmarkerConfig& config)
{
  std::array<char, STATUS_BUFFER_SIZE> statusMessage{};

  if (m_backendHandle != nullptr)
  {
    oasis_mediapipe_backend_destroy_pose_landmarker(m_backendHandle);
    m_backendHandle = nullptr;
  }

  (void)config.loggingName;
  return oasis_mediapipe_backend_create_pose_landmarker(
             config.modelAssetPath.c_str(), config.maxPoses, statusMessage.data(),
             statusMessage.size(), &m_backendHandle) == 0;
}

PoseDetectionResult PoseLandmarker::Detect(const PoseDetectionInput& input)
{
  std::array<char, STATUS_BUFFER_SIZE> statusMessage{};
  OasisPoseLandmarkerOutput backendOutput{};

  if (m_backendHandle == nullptr)
  {
    CopyStatus(statusMessage, "PoseLandmarker is not initialized");
    return {false, GetMessage(statusMessage), {}};
  }

  const bool success = oasis_mediapipe_backend_detect_pose(
                           m_backendHandle, input.data, input.dataSize, input.width, input.height,
                           input.channelCount, input.encoding.c_str(), input.timestampMs,
                           &backendOutput, statusMessage.data(), statusMessage.size()) == 0;

  std::vector<std::vector<PoseLandmark>> poses;
  if (success)
  {
    const int poseCount = std::clamp(backendOutput.poseCount, 0, OASIS_MEDIAPIPE_MAX_POSES);
    poses.reserve(static_cast<std::size_t>(poseCount));

    for (int poseIndex = 0; poseIndex < poseCount; ++poseIndex)
    {
      const OasisPose& backendPose = backendOutput.poses[poseIndex];
      const int landmarkCount =
          std::clamp(backendPose.landmarkCount, 0, OASIS_MEDIAPIPE_POSE_LANDMARK_COUNT);
      std::vector<PoseLandmark> pose;
      pose.reserve(static_cast<std::size_t>(landmarkCount));

      for (int landmarkIndex = 0; landmarkIndex < landmarkCount; ++landmarkIndex)
      {
        const OasisPoseLandmark& landmark = backendPose.landmarks[landmarkIndex];
        pose.push_back({
            landmark.x,
            landmark.y,
            landmark.z,
            landmark.visibility,
            landmark.presence,
        });
      }

      poses.push_back(std::move(pose));
    }
  }

  return {
      success,
      GetMessage(statusMessage),
      std::move(poses),
  };
}
} // namespace oasis_perception::mediapipe_facade
