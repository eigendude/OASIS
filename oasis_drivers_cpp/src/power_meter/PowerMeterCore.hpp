/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <cstdint>
#include <string>

namespace OASIS::PowerMeter
{
/** \brief Runtime configuration for a mux-connected DC power meter */
struct Config
{
  //! Parent Linux I2C bus number, greater than or equal to zero
  int parent_i2c_bus{1};
  //! Qwiic mux 7-bit I2C address, in the inclusive range 0x03 to 0x77
  int mux_address{0x70};
  //! Qwiic mux channel, in the inclusive range 0 to 7
  int mux_channel{0};
  //! Power meter 7-bit I2C address, in the inclusive range 0x03 to 0x77
  int i2c_address{0x60};
  //! Whether startup must resolve the mux child to a Linux I2C adapter
  bool resolve_i2c_adapter{true};
  //! Measurement publication frequency in hertz, greater than zero
  double publish_rate_hz{10.0};
  //! ROS coordinate frame name assigned to headers, or empty when none applies
  //! Nonempty values must name a real coordinate frame in the ROS TF tree
  std::string frame_id;
  //! Simulated DC bus voltage in volts
  double simulated_voltage{0.0};
  //! Simulated signed DC current in amperes
  double simulated_current{0.0};
  //! Simulated voltage variance in squared volts, greater than or equal to zero
  double voltage_variance{0.0};
  //! Simulated current variance in squared amperes, greater than or equal to
  //! zero
  double current_variance{0.0};
  //! Simulated state of the fast hardware overcurrent indication
  bool simulated_overcurrent{false};
};

/** \brief Validity and device state associated with a power-meter sample */
enum class Status : std::uint8_t
{
  //! Valid sample produced by a physical device
  Ok = 0,
  //! Sample produced from configured simulation values
  Simulated = 1,
  //! Sample is valid but older than the allowed freshness interval
  Stale = 2,
  //! Device or transport is not connected
  Disconnected = 3,
  //! Device or processing failure invalidated the sample
  Error = 4,
};

/** \brief Aggregate DC power-meter measurement and validity state */
struct Sample
{
  //! DC bus voltage in volts
  double voltage{0.0};
  //! Voltage variance in squared volts
  double voltage_variance{0.0};
  //! Signed DC current in amperes
  double current{0.0};
  //! Current variance in squared amperes
  double current_variance{0.0};
  //! Signed DC power in watts, computed as voltage times current
  double power{0.0};
  //! First-order power variance in squared watts
  double power_variance{0.0};
  //! Fast hardware overcurrent indication
  bool overcurrent{false};
  //! Validity and device state for the aggregate sample
  Status status{Status::Error};
};

/** \brief Validate configuration and throw std::invalid_argument on failure */
void ValidateConfig(const Config& config);

/** \brief Build one aggregate sample from the configured dummy measurements */
Sample BuildSimulatedSample(const Config& config);
} // namespace OASIS::PowerMeter
