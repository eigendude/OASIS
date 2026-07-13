/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <cstdint>

namespace OASIS::PowerMeter
{
/** \brief Validity and device state associated with a power-meter sample */
enum class Status : std::uint8_t
{
  //! Valid sample produced by a physical device
  Ok,

  //! Sample is valid but older than the allowed freshness interval
  Stale,

  //! Device or transport is not connected
  Disconnected,

  //! Device or processing failure invalidated the sample
  Error,
};

/** \brief Aggregate DC power-meter measurement and validity state */
struct Sample
{
  //! Raw register 0x2A containing vcodes and icodes
  std::uint32_t raw_voltage_current_register{0};

  //! Raw register 0x2C containing pinstant
  std::uint32_t raw_power_register{0};

  //! Raw register 0x2D containing live fault flags
  std::uint32_t raw_fault_register{0};

  //! DC bus voltage in volts
  double voltage{0.0};

  //! Voltage variance in squared volts, or zero when unknown
  double voltage_variance{0.0};

  //! Signed DC current in amperes
  double current{0.0};

  //! Current variance in squared amperes, or zero when unknown
  double current_variance{0.0};

  //! Signed DC active power in watts, decoded from register 0x2C
  double power{0.0};

  //! Power variance in squared watts, or zero when unknown
  double power_variance{0.0};

  //! Fast hardware overcurrent indication
  bool overcurrent{false};

  //! Validity and device state for the aggregate sample
  Status status{Status::Error};
};

/** \brief Validate a legal 7-bit I2C address and throw on failure */
void ValidateI2cAddress(int address, const char* parameter_name);

/** \brief Validate a shared publication rate and throw on failure */
void ValidatePublishRate(double publish_rate_hz);

} // namespace OASIS::PowerMeter
