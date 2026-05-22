/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "localization/ZuptDetector.hpp"

#include <cmath>
#include <optional>
#include <tuple>
#include <vector>

#include <gtest/gtest.h>

using OASIS::Localization::Vector3;
using OASIS::Localization::ZuptDecision;
using OASIS::Localization::ZuptDetector;
using OASIS::Localization::ZuptDetectorConfig;

namespace
{
constexpr Vector3 kQuietGyroRads{{0.0, 0.0, 0.0}};
constexpr Vector3 kLoudGyroRads{{0.2, 0.0, 0.0}};
constexpr Vector3 kQuietAccelMps2{{0.0, 0.0, 0.0}};
constexpr Vector3 kLoudAccelMps2{{0.3, 0.0, 0.0}};

ZuptDetector MakeDetector()
{
  ZuptDetectorConfig config;
  config.min_stationary_sec = 0.18;
  config.min_moving_sec = 0.01;
  config.stationary_linear_velocity_sigma_mps = 0.06;
  config.stationary_angular_velocity_sigma_rads = 0.05;
  config.moving_linear_variance_mps2 = 1.0e6;
  config.moving_angular_variance_rads2 = 5.0e5;
  return ZuptDetector(config);
}

ZuptDecision Accepted(const std::optional<ZuptDecision>& decision)
{
  EXPECT_TRUE(decision.has_value());
  return *decision;
}
} // namespace

TEST(ZuptDetector, EntersStationaryAfterQuietDwell)
{
  ZuptDetector detector = MakeDetector();

  const ZuptDecision first_result = Accepted(detector.Update(0.0, kQuietGyroRads, kQuietAccelMps2));
  const ZuptDecision second_result =
      Accepted(detector.Update(0.18, kQuietGyroRads, kQuietAccelMps2));

  EXPECT_FALSE(first_result.stationary);
  EXPECT_EQ(first_result.reason, "enter_candidate_started");
  EXPECT_TRUE(second_result.stationary);
  EXPECT_EQ(second_result.reason, "stationary_asserted");
}

TEST(ZuptDetector, QuietDwellResetsWhenSampleExceedsEnterThreshold)
{
  ZuptDetector detector = MakeDetector();

  Accepted(detector.Update(0.0, kQuietGyroRads, kQuietAccelMps2));
  const ZuptDecision interrupted_result =
      Accepted(detector.Update(0.10, Vector3{{0.07, 0.0, 0.0}}, kQuietAccelMps2));
  const ZuptDecision final_result =
      Accepted(detector.Update(0.28, kQuietGyroRads, kQuietAccelMps2));

  EXPECT_FALSE(interrupted_result.stationary);
  EXPECT_EQ(interrupted_result.reason, "moving");
  EXPECT_FALSE(final_result.stationary);
  EXPECT_EQ(final_result.reason, "enter_candidate_started");
}

TEST(ZuptDetector, MovingVibrationDoesNotFalseAssertStationary)
{
  ZuptDetector detector = MakeDetector();
  const std::vector<std::tuple<double, Vector3, Vector3>> samples{
      {0.00, Vector3{{0.07, 0.0, 0.0}}, Vector3{{0.20, 0.0, 0.0}}},
      {0.05, kQuietGyroRads, kQuietAccelMps2},
      {0.10, Vector3{{0.10, 0.0, 0.0}}, Vector3{{0.24, 0.0, 0.0}}},
      {0.15, kQuietGyroRads, kQuietAccelMps2},
      {0.20, Vector3{{0.12, 0.0, 0.0}}, Vector3{{0.22, 0.0, 0.0}}},
      {0.25, kQuietGyroRads, kQuietAccelMps2},
      {0.30, Vector3{{0.10, 0.0, 0.0}}, Vector3{{0.25, 0.0, 0.0}}},
      {0.35, kQuietGyroRads, kQuietAccelMps2},
  };

  std::vector<ZuptDecision> decisions;
  for (const auto& [timestamp_sec, gyro_rads, accel_mps2] : samples)
  {
    decisions.push_back(Accepted(detector.Update(timestamp_sec, gyro_rads, accel_mps2)));
  }

  for (const ZuptDecision& decision : decisions)
  {
    EXPECT_FALSE(decision.stationary);
  }

  EXPECT_TRUE(decisions.back().reason == "moving" ||
              decisions.back().reason == "enter_candidate_started");
}

TEST(ZuptDetector, ExitsStationaryQuicklyWithGyroThresholdBreach)
{
  ZuptDetector detector = MakeDetector();

  Accepted(detector.Update(0.0, kQuietGyroRads, kQuietAccelMps2));
  Accepted(detector.Update(0.18, kQuietGyroRads, kQuietAccelMps2));
  const ZuptDecision result = Accepted(detector.Update(0.19, kLoudGyroRads, kQuietAccelMps2));

  EXPECT_FALSE(result.stationary);
  EXPECT_EQ(result.reason, "stationary_cleared");
  EXPECT_DOUBLE_EQ(result.linear_zupt_variance_mps2, 1.0e6);
  EXPECT_DOUBLE_EQ(result.angular_zupt_variance_rads2, 5.0e5);
}

TEST(ZuptDetector, ExitsStationaryQuicklyWithAccelThresholdBreach)
{
  ZuptDetector detector = MakeDetector();

  Accepted(detector.Update(0.0, kQuietGyroRads, kQuietAccelMps2));
  Accepted(detector.Update(0.18, kQuietGyroRads, kQuietAccelMps2));
  const ZuptDecision result = Accepted(detector.Update(0.19, kQuietGyroRads, kLoudAccelMps2));

  EXPECT_FALSE(result.stationary);
  EXPECT_EQ(result.reason, "stationary_cleared");
}

TEST(ZuptDetector, ExitHappensFasterThanEnter)
{
  ZuptDetector detector = MakeDetector();

  const ZuptDecision initial_result =
      Accepted(detector.Update(0.0, kQuietGyroRads, kQuietAccelMps2));
  const ZuptDecision pending_enter_result =
      Accepted(detector.Update(0.01, kQuietGyroRads, kQuietAccelMps2));
  const ZuptDecision stationary_result =
      Accepted(detector.Update(0.18, kQuietGyroRads, kQuietAccelMps2));
  const ZuptDecision cleared_result =
      Accepted(detector.Update(0.19, kLoudGyroRads, kQuietAccelMps2));

  EXPECT_EQ(initial_result.reason, "enter_candidate_started");
  EXPECT_FALSE(pending_enter_result.stationary);
  EXPECT_EQ(pending_enter_result.reason, "enter_candidate_pending");
  EXPECT_TRUE(stationary_result.stationary);
  EXPECT_FALSE(cleared_result.stationary);
  EXPECT_FALSE(detector.State().stationary);
  EXPECT_EQ(detector.State().last_reason, "stationary_cleared");
}

TEST(ZuptDetector, StationaryVariancesAreSigmaSquared)
{
  ZuptDetector detector = MakeDetector();

  Accepted(detector.Update(0.0, kQuietGyroRads, kQuietAccelMps2));
  const ZuptDecision result = Accepted(detector.Update(0.18, kQuietGyroRads, kQuietAccelMps2));

  EXPECT_TRUE(result.stationary);
  EXPECT_DOUBLE_EQ(result.linear_zupt_variance_mps2, 0.06 * 0.06);
  EXPECT_DOUBLE_EQ(result.angular_zupt_variance_rads2, 0.05 * 0.05);
}

TEST(ZuptDetector, MovingVariancesUseMovingDefaults)
{
  ZuptDetector detector = MakeDetector();

  const ZuptDecision result = Accepted(detector.Update(0.0, kLoudGyroRads, kLoudAccelMps2));

  EXPECT_FALSE(result.stationary);
  EXPECT_DOUBLE_EQ(result.linear_zupt_variance_mps2, 1.0e6);
  EXPECT_DOUBLE_EQ(result.angular_zupt_variance_rads2, 5.0e5);
}

TEST(ZuptDetector, InvalidImuSampleReportsInvalidWithoutChangingStationarity)
{
  ZuptDetector detector = MakeDetector();

  const ZuptDecision initial_result =
      Accepted(detector.Update(0.0, kQuietGyroRads, kQuietAccelMps2));
  const ZuptDecision invalid_result =
      Accepted(detector.Update(0.1, Vector3{{std::nan(""), 0.0, 0.0}}, kQuietAccelMps2));

  EXPECT_FALSE(initial_result.stationary);
  EXPECT_FALSE(invalid_result.stationary);
  EXPECT_EQ(invalid_result.reason, "invalid_imu_sample");
  ASSERT_TRUE(detector.State().last_timestamp_sec.has_value());
  EXPECT_DOUBLE_EQ(*detector.State().last_timestamp_sec, 0.1);
}

TEST(ZuptDetector, InvalidTimestampReturnsNoDecision)
{
  ZuptDetector detector = MakeDetector();

  const std::optional<ZuptDecision> decision =
      detector.Update(std::nan(""), kQuietGyroRads, kQuietAccelMps2);

  EXPECT_FALSE(decision.has_value());
  EXPECT_EQ(detector.State().last_reason, "invalid_timestamp");
}

TEST(ZuptDetector, NonMonotonicTimestampReturnsNoDecisionAndClearsCandidates)
{
  ZuptDetector detector = MakeDetector();

  Accepted(detector.Update(0.0, kQuietGyroRads, kQuietAccelMps2));
  ASSERT_TRUE(detector.State().enter_candidate_start_sec.has_value());
  EXPECT_DOUBLE_EQ(*detector.State().enter_candidate_start_sec, 0.0);

  const std::optional<ZuptDecision> decision =
      detector.Update(-0.1, kQuietGyroRads, kQuietAccelMps2);

  EXPECT_FALSE(decision.has_value());
  EXPECT_EQ(detector.State().last_reason, "non_monotonic_timestamp");
  EXPECT_FALSE(detector.State().enter_candidate_start_sec.has_value());
  EXPECT_FALSE(detector.State().exit_candidate_start_sec.has_value());
}
