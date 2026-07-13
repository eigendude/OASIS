/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "power_meter/PowerMeterCore.hpp"

#include <limits>

#include <gtest/gtest.h>

using OASIS::PowerMeter::ValidateI2cAddress;
using OASIS::PowerMeter::ValidatePublishRate;

TEST(PowerMeterCore, RejectsInvalidDeviceAddress)
{
  EXPECT_NO_THROW(ValidateI2cAddress(0x03, "address"));
  EXPECT_NO_THROW(ValidateI2cAddress(0x77, "address"));
  EXPECT_THROW(ValidateI2cAddress(0x02, "address"), std::invalid_argument);
  EXPECT_THROW(ValidateI2cAddress(0x78, "address"), std::invalid_argument);
}

TEST(PowerMeterCore, ValidatesSharedPublishRate)
{
  EXPECT_NO_THROW(ValidatePublishRate(10.0));
  EXPECT_THROW(ValidatePublishRate(0.0), std::invalid_argument);
  EXPECT_THROW(ValidatePublishRate(std::numeric_limits<double>::quiet_NaN()),
               std::invalid_argument);
}
