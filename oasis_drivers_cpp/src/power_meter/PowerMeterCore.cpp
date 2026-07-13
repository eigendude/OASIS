/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "PowerMeterCore.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

void OASIS::PowerMeter::ValidateI2cAddress(int address, const char* parameter_name)
{
  if (address < 0x03 || address > 0x77)
    throw std::invalid_argument(std::string(parameter_name) + " must be in [0x03, 0x77]");
}

void OASIS::PowerMeter::ValidatePublishRate(double publish_rate_hz)
{
  if (!std::isfinite(publish_rate_hz) || publish_rate_hz <= 0.0)
    throw std::invalid_argument("publish_rate_hz must be finite and greater than zero");
}
