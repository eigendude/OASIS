/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  This file is derived from the MediaPipe project.
 *  Copyright (C) 2018-2025 The MediaPipe Authors
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "HelloWorld.h"

#include <iostream>

#include <absl/status/status.h>
#include <glog/logging.h>
#include <mediapipe/framework/calculator_graph.h>
#include <mediapipe/framework/port/parse_text_proto.h>
#include <mediapipe/framework/port/status.h>

using namespace oasis_perception;
using namespace mediapipe;

namespace
{
constexpr const char* PROGRAM_NAME = "hello_world";
}

HelloWorld::HelloWorld() = default;

HelloWorld::~HelloWorld() = default;

void HelloWorld::Initialize()
{
  google::InitGoogleLogging(PROGRAM_NAME);

  // Route to stderr
  FLAGS_logtostderr = true;
  FLAGS_alsologtostderr = true;

  // Full verbosity
  FLAGS_v = 3;
}

void HelloWorld::Run()
{
  if (!PrintHelloWorld())
    std::cerr << "Graph failed" << std::endl;
}

bool HelloWorld::PrintHelloWorld()
{
  // Configures a simple graph, which concatenates 2 PassThroughCalculators
  CalculatorGraphConfig config = ParseTextProtoOrDie<CalculatorGraphConfig>(R"pb(
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

  CalculatorGraph graph;

  // Initialize the graph with the config
  const absl::Status initializeStatus = graph.Initialize(config);
  if (!initializeStatus.ok())
  {
    std::cerr << "Graph initialization failed" << std::endl;
    return false;
  }

  // Add an output stream "out" to the graph
  absl::StatusOr<OutputStreamPoller> pollerResult = graph.AddOutputStreamPoller("out");
  if (!pollerResult.status().ok())
  {
    std::cerr << "Graph add output stream poller failed" << std::endl;
    return false;
  }
  OutputStreamPoller& poller = pollerResult.value();

  // Start the graph
  const absl::Status startStatus = graph.StartRun({});
  if (!startStatus.ok())
  {
    std::cerr << "Graph start failed" << std::endl;
    return false;
  }

  // Give 10 input packets that contains the same string "Hello World!"
  for (int i = 0; i < 10; ++i)
  {
    if (!graph
             .AddPacketToInputStream("in", MakePacket<std::string>("Hello World!").At(Timestamp(i)))
             .ok())
    {
      std::cerr << "Graph add packet failed" << std::endl;
      return false;
    }
  }

  // Close the input stream "in"
  const absl::Status closeStatus = graph.CloseInputStream("in");
  if (!closeStatus.ok())
  {
    std::cerr << "Graph close input stream failed" << std::endl;
    return false;
  }

  // Get the output packets string
  mediapipe::Packet packet;
  while (poller.Next(&packet))
  {
    std::cout << "Output packet: " << packet.Get<std::string>() << std::endl;
  }

  return graph.WaitUntilDone().ok();
}
