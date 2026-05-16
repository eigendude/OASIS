/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/Bno086DiagnosticsPolicy.hpp"

#include <gtest/gtest.h>

using namespace OASIS::IMU::BNO086;

TEST(Bno086DiagnosticsPolicy, parsesAllowedLogLevels)
{
  EXPECT_EQ(ParseBno086DiagnosticsLogLevel("debug"), Bno086DiagnosticsLogLevel::Debug);
  EXPECT_EQ(ParseBno086DiagnosticsLogLevel("info"), Bno086DiagnosticsLogLevel::Info);
  EXPECT_EQ(ParseBno086DiagnosticsLogLevel("warn"), Bno086DiagnosticsLogLevel::Warn);
  EXPECT_EQ(ParseBno086DiagnosticsLogLevel("off"), Bno086DiagnosticsLogLevel::Off);
}

TEST(Bno086DiagnosticsPolicy, rejectsUnknownLogLevels)
{
  EXPECT_FALSE(ParseBno086DiagnosticsLogLevel("trace").has_value());
  EXPECT_FALSE(ParseBno086DiagnosticsLogLevel("DEBUG").has_value());
  EXPECT_FALSE(ParseBno086DiagnosticsLogLevel("").has_value());
}

TEST(Bno086DiagnosticsPolicy, offSuppressesDiagnostics)
{
  EXPECT_FALSE(ShouldEmitBno086Diagnostics(Bno086DiagnosticsLogLevel::Off, false));
  EXPECT_FALSE(ShouldEmitBno086Diagnostics(Bno086DiagnosticsLogLevel::Off, true));
}

TEST(Bno086DiagnosticsPolicy, warnEmitsOnlyWhenUnhealthy)
{
  EXPECT_FALSE(ShouldEmitBno086Diagnostics(Bno086DiagnosticsLogLevel::Warn, false));
  EXPECT_TRUE(ShouldEmitBno086Diagnostics(Bno086DiagnosticsLogLevel::Warn, true));
}

TEST(Bno086DiagnosticsPolicy, debugAndInfoEmitPeriodicDiagnostics)
{
  EXPECT_TRUE(ShouldEmitBno086Diagnostics(Bno086DiagnosticsLogLevel::Debug, false));
  EXPECT_TRUE(ShouldEmitBno086Diagnostics(Bno086DiagnosticsLogLevel::Info, false));
}
