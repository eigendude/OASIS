/*
 *  Copyright (C) 2021-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "LogUtils.h"

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rcutils/error_handling.h>
#include <rcutils/logging.h>

using namespace OASIS;
using namespace UTILS;

rclcpp::Logger LogUtils::InitializeLogging(const std::shared_ptr<rclcpp::Node>& node)
{
  rclcpp::Logger logger = node->get_logger();

  RCLCPP_INFO(logger, "Setting log severity threshold to DEBUG");

  auto ret = rcutils_logging_set_logger_level(logger.get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
  if (ret != RCUTILS_RET_OK)
  {
    RCLCPP_ERROR(logger, "Error setting severity: %s", rcutils_get_error_string().str);
    rcutils_reset_error();
  }

  return logger;
}
