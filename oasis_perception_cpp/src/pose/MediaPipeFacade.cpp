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

#ifndef OASIS_MEDIAPIPE_BACKEND_AVAILABLE
#error "OASIS_MEDIAPIPE_BACKEND_AVAILABLE must be defined for MediaPipeFacade.cpp"
#endif

namespace oasis_perception::mediapipe_facade
{
extern "C"
{
  int oasis_mediapipe_backend_run_hello_world(char* statusMessage,
                                              std::size_t statusMessageSize,
                                              int* outputPacketCount);

  int oasis_mediapipe_backend_initialize_pose_landmarker(const char* loggingName,
                                                         char* statusMessage,
                                                         std::size_t statusMessageSize);

  int oasis_mediapipe_backend_detect_pose_stub(int width,
                                               int height,
                                               int channelCount,
                                               const char* encoding,
                                               long long* observedScalarCount,
                                               char* statusMessage,
                                               std::size_t statusMessageSize);
}

namespace
{
constexpr std::size_t STATUS_BUFFER_SIZE = 512;

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

PoseLandmarker::~PoseLandmarker() = default;

bool PoseLandmarker::Initialize(const PoseLandmarkerConfig& config)
{
  std::array<char, STATUS_BUFFER_SIZE> statusMessage{};

  m_initialized = oasis_mediapipe_backend_initialize_pose_landmarker(
                      config.loggingName.c_str(), statusMessage.data(), statusMessage.size()) == 0;

  return m_initialized;
}

PoseDetectionStubResult PoseLandmarker::DetectStub(const PoseDetectionStubInput& input)
{
  std::array<char, STATUS_BUFFER_SIZE> statusMessage{};
  long long observedScalarCount = 0;

  if (!m_initialized)
  {
    CopyStatus(statusMessage, "PoseLandmarker is not initialized");
    return {false, GetMessage(statusMessage), observedScalarCount};
  }

  const bool success = oasis_mediapipe_backend_detect_pose_stub(
                           input.width, input.height, input.channelCount, input.encoding.c_str(),
                           &observedScalarCount, statusMessage.data(), statusMessage.size()) == 0;

  return {
      success,
      GetMessage(statusMessage),
      observedScalarCount,
  };
}
} // namespace oasis_perception::mediapipe_facade
