/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "mediapipe/framework/calculator_graph.h"
#include "mediapipe/framework/port/parse_text_proto.h"
#include "mediapipe/framework/port/status.h"
#include "mediapipe/tasks/c/core/common.h"
#include "mediapipe/tasks/c/vision/core/image.h"
#include "mediapipe/tasks/c/vision/pose_landmarker/pose_landmarker.h"

#include <algorithm>
#include <cstddef>
#include <cstring>
#include <string>

namespace
{
constexpr int OASIS_MEDIAPIPE_MAX_POSES = 5;
constexpr int OASIS_MEDIAPIPE_POSE_LANDMARK_COUNT = 33;

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

struct OasisPoseLandmarkerHandle
{
  MpPoseLandmarkerPtr landmarker = nullptr;
  int maxPoses = 1;
};

void CopyStatus(const std::string& message, char* statusMessage, std::size_t statusMessageSize)
{
  if (statusMessage == nullptr || statusMessageSize == 0)
    return;

  const std::size_t copySize = std::min(statusMessageSize - 1, message.size());
  std::memcpy(statusMessage, message.data(), copySize);
  statusMessage[copySize] = '\0';
}

std::string ConsumeError(char* errorMessage)
{
  if (errorMessage == nullptr)
    return {};

  std::string message{errorMessage};
  MpErrorFree(errorMessage);
  return message;
}

MpImageFormat GetImageFormat(int channelCount)
{
  if (channelCount == 3)
    return kMpImageFormatSrgb;

  if (channelCount == 4)
    return kMpImageFormatSrgba;

  return kMpImageFormatUnknown;
}

void CopyPoseResult(const PoseLandmarkerResult& result, OasisPoseLandmarkerOutput* output)
{
  if (output == nullptr)
    return;

  *output = {};

  const int poseCount =
      std::min<int>(static_cast<int>(result.pose_landmarks_count), OASIS_MEDIAPIPE_MAX_POSES);
  output->poseCount = poseCount;

  for (int poseIndex = 0; poseIndex < poseCount; ++poseIndex)
  {
    const NormalizedLandmarks& sourcePose = result.pose_landmarks[poseIndex];
    OasisPose& destinationPose = output->poses[poseIndex];
    const int landmarkCount = std::min<int>(static_cast<int>(sourcePose.landmarks_count),
                                            OASIS_MEDIAPIPE_POSE_LANDMARK_COUNT);
    destinationPose.landmarkCount = landmarkCount;

    for (int landmarkIndex = 0; landmarkIndex < landmarkCount; ++landmarkIndex)
    {
      const NormalizedLandmark& sourceLandmark = sourcePose.landmarks[landmarkIndex];
      OasisPoseLandmark& destinationLandmark = destinationPose.landmarks[landmarkIndex];
      destinationLandmark.x = sourceLandmark.x;
      destinationLandmark.y = sourceLandmark.y;
      destinationLandmark.z = sourceLandmark.z;
      destinationLandmark.visibility =
          sourceLandmark.has_visibility ? sourceLandmark.visibility : 0.0F;
      destinationLandmark.presence = sourceLandmark.has_presence ? sourceLandmark.presence : 0.0F;
    }
  }
}
} // namespace

extern "C" int oasis_mediapipe_backend_run_hello_world(char* statusMessage,
                                                       std::size_t statusMessageSize,
                                                       int* outputPacketCount)
{
  if (outputPacketCount == nullptr)
  {
    CopyStatus("Output packet count pointer is null", statusMessage, statusMessageSize);
    return 1;
  }

  *outputPacketCount = 0;

  mediapipe::CalculatorGraphConfig config =
      mediapipe::ParseTextProtoOrDie<mediapipe::CalculatorGraphConfig>(R"pb(
        input_stream: "in"
        output_stream: "out"
        node {
          calculator: "PassThroughCalculator"
          input_stream: "in"
          output_stream: "out1"
        }
        node {
          calculator: "PassThroughCalculator"
          input_stream: "out1"
          output_stream: "out"
        }
      )pb");

  mediapipe::CalculatorGraph graph;

  absl::Status status = graph.Initialize(config);
  if (!status.ok())
  {
    CopyStatus(std::string{"Graph initialization failed: "} + std::string{status.message()},
               statusMessage, statusMessageSize);
    return 1;
  }

  absl::StatusOr<mediapipe::OutputStreamPoller> pollerResult = graph.AddOutputStreamPoller("out");
  if (!pollerResult.status().ok())
  {
    CopyStatus(std::string{"Graph poller setup failed: "} +
                   std::string{pollerResult.status().message()},
               statusMessage, statusMessageSize);
    return 1;
  }

  mediapipe::OutputStreamPoller& poller = pollerResult.value();

  status = graph.StartRun({});
  if (!status.ok())
  {
    CopyStatus(std::string{"Graph start failed: "} + std::string{status.message()}, statusMessage,
               statusMessageSize);
    return 1;
  }

  for (int index = 0; index < 10; ++index)
  {
    status = graph.AddPacketToInputStream(
        "in", mediapipe::MakePacket<std::string>("Hello World!").At(mediapipe::Timestamp(index)));
    if (!status.ok())
    {
      CopyStatus(std::string{"Graph input failed: "} + std::string{status.message()}, statusMessage,
                 statusMessageSize);
      return 1;
    }
  }

  status = graph.CloseInputStream("in");
  if (!status.ok())
  {
    CopyStatus(std::string{"Graph input close failed: "} + std::string{status.message()},
               statusMessage, statusMessageSize);
    return 1;
  }

  mediapipe::Packet packet;
  while (poller.Next(&packet))
    ++(*outputPacketCount);

  status = graph.WaitUntilDone();
  if (!status.ok())
  {
    CopyStatus(std::string{"Graph wait failed: "} + std::string{status.message()}, statusMessage,
               statusMessageSize);
    return 1;
  }

  CopyStatus("MediaPipe backend hello-world graph completed", statusMessage, statusMessageSize);
  return 0;
}

extern "C" int oasis_mediapipe_backend_create_pose_landmarker(const char* modelAssetPath,
                                                              int maxPoses,
                                                              char* statusMessage,
                                                              std::size_t statusMessageSize,
                                                              void** handle)
{
  if (handle == nullptr)
  {
    CopyStatus("Pose landmarker handle pointer is null", statusMessage, statusMessageSize);
    return 1;
  }

  *handle = nullptr;

  if (modelAssetPath == nullptr || std::strlen(modelAssetPath) == 0)
  {
    CopyStatus("Pose landmarker model asset path is empty", statusMessage, statusMessageSize);
    return 1;
  }

  auto* wrapper = new OasisPoseLandmarkerHandle();
  wrapper->maxPoses = std::clamp(maxPoses, 1, OASIS_MEDIAPIPE_MAX_POSES);

  PoseLandmarkerOptions options = {};
  options.base_options.model_asset_path = modelAssetPath;
  options.base_options.delegate = CPU;
  options.base_options.host_environment = HOST_ENVIRONMENT_UNKNOWN;
  options.base_options.host_system = HOST_SYSTEM_LINUX;
  options.running_mode = IMAGE;
  options.num_poses = wrapper->maxPoses;
  options.min_pose_detection_confidence = 0.5F;
  options.min_pose_presence_confidence = 0.5F;
  options.min_tracking_confidence = 0.5F;
  options.output_segmentation_masks = false;

  char* errorMessage = nullptr;
  const MpStatus status = MpPoseLandmarkerCreate(&options, &wrapper->landmarker, &errorMessage);
  if (status != 0 || wrapper->landmarker == nullptr)
  {
    const std::string error = ConsumeError(errorMessage);
    delete wrapper;
    CopyStatus("MediaPipe backend pose landmarker create failed: " + error, statusMessage,
               statusMessageSize);
    return 1;
  }

  *handle = wrapper;
  CopyStatus("MediaPipe backend pose landmarker created", statusMessage, statusMessageSize);
  return 0;
}

extern "C" int oasis_mediapipe_backend_destroy_pose_landmarker(void* handle)
{
  auto* wrapper = static_cast<OasisPoseLandmarkerHandle*>(handle);
  if (wrapper == nullptr)
    return 0;

  if (wrapper->landmarker != nullptr)
  {
    char* errorMessage = nullptr;
    MpPoseLandmarkerClose(wrapper->landmarker, &errorMessage);
    ConsumeError(errorMessage);
    wrapper->landmarker = nullptr;
  }

  delete wrapper;
  return 0;
}

extern "C" int oasis_mediapipe_backend_detect_pose(void* handle,
                                                   const unsigned char* imageData,
                                                   std::size_t imageDataSize,
                                                   int width,
                                                   int height,
                                                   int channelCount,
                                                   const char* encoding,
                                                   long long timestampMs,
                                                   OasisPoseLandmarkerOutput* output,
                                                   char* statusMessage,
                                                   std::size_t statusMessageSize)
{
  auto* wrapper = static_cast<OasisPoseLandmarkerHandle*>(handle);
  if (wrapper == nullptr || wrapper->landmarker == nullptr)
  {
    CopyStatus("Pose landmarker handle is null", statusMessage, statusMessageSize);
    return 1;
  }

  if (output == nullptr)
  {
    CopyStatus("Pose landmarker output pointer is null", statusMessage, statusMessageSize);
    return 1;
  }

  *output = {};

  if (imageData == nullptr || imageDataSize == 0)
  {
    CopyStatus("Pose landmarker image data is empty", statusMessage, statusMessageSize);
    return 1;
  }

  const MpImageFormat imageFormat = GetImageFormat(channelCount);
  if (imageFormat == kMpImageFormatUnknown)
  {
    const std::string imageEncoding = encoding != nullptr ? encoding : "";
    CopyStatus("Unsupported pose landmarker image encoding: " + imageEncoding, statusMessage,
               statusMessageSize);
    return 1;
  }

  const std::size_t expectedSize = static_cast<std::size_t>(width) *
                                   static_cast<std::size_t>(height) *
                                   static_cast<std::size_t>(channelCount);
  if (width <= 0 || height <= 0 || imageDataSize < expectedSize)
  {
    CopyStatus("Pose landmarker image dimensions or data size are invalid", statusMessage,
               statusMessageSize);
    return 1;
  }

  MpImagePtr image = nullptr;
  char* errorMessage = nullptr;
  MpStatus status =
      MpImageCreateFromUint8Data(imageFormat, width, height, imageData,
                                 static_cast<int>(imageDataSize), &image, &errorMessage);
  if (status != 0 || image == nullptr)
  {
    const std::string error = ConsumeError(errorMessage);
    CopyStatus("MediaPipe image creation failed: " + error, statusMessage, statusMessageSize);
    return 1;
  }

  PoseLandmarkerResult result = {};
  status = MpPoseLandmarkerDetectImage(wrapper->landmarker, image, nullptr, &result, &errorMessage);
  MpImageFree(image);

  if (status != 0)
  {
    const std::string error = ConsumeError(errorMessage);
    MpPoseLandmarkerCloseResult(&result);
    CopyStatus("MediaPipe backend pose detection failed: " + error, statusMessage,
               statusMessageSize);
    return 1;
  }

  (void)timestampMs;
  CopyPoseResult(result, output);
  MpPoseLandmarkerCloseResult(&result);

  CopyStatus("MediaPipe backend pose detection completed", statusMessage, statusMessageSize);
  return 0;
}
