/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/utils/Bno086FeatureSummary.hpp"

#include "imu/bno086/utils/Bno086ReportUtils.hpp"

#include <algorithm>
#include <sstream>

namespace OASIS::IMU::BNO086
{
namespace
{
std::optional<FeatureResponse> FindFeatureResponse(const std::vector<FeatureResponse>& responses,
                                                   ReportId report_id)
{
  const auto it =
      std::find_if(responses.begin(), responses.end(), [report_id](const FeatureResponse& response)
                   { return response.report_id == report_id; });
  if (it == responses.end())
    return std::nullopt;

  return *it;
}
} // namespace

std::string BuildFeatureSummaryLine(const FeatureConfiguration& configuration,
                                    const std::optional<FeatureResponse>& response)
{
  std::ostringstream oss;
  oss << ReportName(configuration.report_id)
      << " requested_interval_us=" << configuration.requested_interval_us
      << " requested_batch_us=" << configuration.requested_batch_interval_us;

  if (!response.has_value())
  {
    oss << " status=missing_response";
    return oss.str();
  }

  oss << " actual_interval_us=" << response->report_interval_us
      << " actual_batch_us=" << response->batch_interval_us
      << " batching_active=" << (IsFeatureBatchingActive(*response) ? "true" : "false");
  return oss.str();
}

std::string BuildFeatureSummary(const std::vector<FeatureConfiguration>& configurations,
                                const std::vector<FeatureResponse>& responses)
{
  std::ostringstream oss;
  if (responses.size() >= configurations.size())
  {
    oss << "BNO086 feature acceptance summary:";
  }
  else
  {
    oss << "BNO086 feature acceptance summary incomplete: received=" << responses.size()
        << " expected=" << configurations.size();
  }

  for (const FeatureConfiguration& configuration : configurations)
  {
    oss << "\n  "
        << BuildFeatureSummaryLine(configuration,
                                   FindFeatureResponse(responses, configuration.report_id));
  }

  return oss.str();
}
} // namespace OASIS::IMU::BNO086
