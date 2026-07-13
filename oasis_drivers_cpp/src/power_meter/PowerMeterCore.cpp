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

using OASIS::PowerMeter::Config;
using OASIS::PowerMeter::Sample;
using OASIS::PowerMeter::Status;

void OASIS::PowerMeter::ValidateConfig(const Config& config)
{
  if (config.parent_i2c_bus < 0)
    throw std::invalid_argument("parent_i2c_bus must be nonnegative");
  if (config.mux_address < 0x03 || config.mux_address > 0x77)
    throw std::invalid_argument("mux_address must be in [0x03, 0x77]");
  if (config.mux_channel < 0 || config.mux_channel > 7)
    throw std::invalid_argument("mux_channel must be in [0, 7]");
  if (config.i2c_address < 0x03 || config.i2c_address > 0x77)
    throw std::invalid_argument("i2c_address must be in [0x03, 0x77]");
  if (!std::isfinite(config.publish_rate_hz) || config.publish_rate_hz <= 0.0)
    throw std::invalid_argument("publish_rate_hz must be finite and greater than zero");
  if (!std::isfinite(config.simulated_voltage))
    throw std::invalid_argument("simulated_voltage must be finite");
  if (!std::isfinite(config.simulated_current))
    throw std::invalid_argument("simulated_current must be finite");
  if (!std::isfinite(config.voltage_variance) || config.voltage_variance < 0.0)
    throw std::invalid_argument("voltage_variance must be finite and nonnegative");
  if (!std::isfinite(config.current_variance) || config.current_variance < 0.0)
    throw std::invalid_argument("current_variance must be finite and nonnegative");
}

Sample OASIS::PowerMeter::BuildSimulatedSample(const Config& config)
{
  Sample sample;
  sample.voltage = config.simulated_voltage;
  sample.voltage_variance = config.voltage_variance;
  sample.current = config.simulated_current;
  sample.current_variance = config.current_variance;
  sample.power = config.simulated_voltage * config.simulated_current;

  // For independent voltage V and current I estimates, first-order error
  // propagation gives Var(VI) = I^2 Var(V) + V^2 Var(I)
  sample.power_variance =
      config.simulated_current * config.simulated_current * config.voltage_variance +
      config.simulated_voltage * config.simulated_voltage * config.current_variance;

  sample.overcurrent = config.simulated_overcurrent;
  sample.status = Status::Simulated;
  return sample;
}
