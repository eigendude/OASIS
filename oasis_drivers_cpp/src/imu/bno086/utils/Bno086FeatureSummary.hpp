/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "imu/bno086/sh2/Bno086Reports.hpp"

#include <optional>
#include <string>
#include <vector>

namespace OASIS::IMU::BNO086
{
std::string BuildFeatureSummaryLine(const FeatureConfiguration& configuration,
                                    const std::optional<FeatureResponse>& response);

std::string BuildFeatureSummary(const std::vector<FeatureConfiguration>& configurations,
                                const std::vector<FeatureResponse>& responses);
} // namespace OASIS::IMU::BNO086
