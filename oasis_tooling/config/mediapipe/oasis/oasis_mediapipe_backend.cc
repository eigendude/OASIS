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

#include <algorithm>
#include <cstddef>
#include <cstring>
#include <string>

namespace
{
void CopyStatus(const std::string& message, char* statusMessage, std::size_t statusMessageSize)
{
  if (statusMessage == nullptr || statusMessageSize == 0)
    return;

  const std::size_t copySize = std::min(statusMessageSize - 1, message.size());
  std::memcpy(statusMessage, message.data(), copySize);
  statusMessage[copySize] = '\0';
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

extern "C" int oasis_mediapipe_backend_initialize_pose_landmarker(const char* loggingName,
                                                                  char* statusMessage,
                                                                  std::size_t statusMessageSize)
{
  const std::string caller = loggingName != nullptr ? loggingName : "";
  CopyStatus("MediaPipe backend pose landmarker initialized for " + caller, statusMessage,
             statusMessageSize);
  return 0;
}

extern "C" int oasis_mediapipe_backend_detect_pose_stub(int width,
                                                        int height,
                                                        int channelCount,
                                                        const char* encoding,
                                                        long long* observedScalarCount,
                                                        char* statusMessage,
                                                        std::size_t statusMessageSize)
{
  if (observedScalarCount == nullptr)
  {
    CopyStatus("Observed scalar count pointer is null", statusMessage, statusMessageSize);
    return 1;
  }

  if (width < 0 || height < 0 || channelCount < 0)
  {
    CopyStatus("Image dimensions and channels must be non-negative", statusMessage,
               statusMessageSize);
    return 1;
  }

  *observedScalarCount = static_cast<long long>(width) * static_cast<long long>(height) *
                         static_cast<long long>(channelCount);

  const std::string imageEncoding = encoding != nullptr ? encoding : "";
  CopyStatus("MediaPipe backend pose stub observed " + imageEncoding + " image", statusMessage,
             statusMessageSize);
  return 0;
}
