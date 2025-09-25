/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "OpticalFlowNode.h"

#include "video/OpticalFlow.h"

#include <rclcpp/node.hpp>

using namespace OASIS;
using namespace ROS;

OpticalFlowNode::OpticalFlowNode(rclcpp::Node& node)
  : m_node(node), m_opticalFlow(std::make_unique<VIDEO::OpticalFlow>())
{
}

OpticalFlowNode::~OpticalFlowNode() = default;

bool OpticalFlowNode::Initialize()
{
  return m_opticalFlow->Initialize(640, 480); // TODO

  //m_opticalFlow->Run();

  return true;
}

void OpticalFlowNode::Deinitialize()
{
  m_opticalFlow->Deinitialize();
}
