/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <string>

namespace OASIS::PowerMeter
{
/** \brief Outcome of a non-destructive I2C device probe */
enum class I2cProbeStatus
{
  //! The addressed device acknowledged and completed the register read
  Present,

  //! The adapter reported an expected address-level NACK
  NotPresent,

  //! Adapter setup or the transfer failed for another reason
  Error,
};

/** \brief Result of probing one address through one Linux I2C adapter */
struct I2cProbeResult
{
  //! Classified probe outcome used by discovery
  I2cProbeStatus status{I2cProbeStatus::Error};

  //! Human-readable failure detail, empty for successful probes and NACKs
  std::string error;
};

/** \brief Interface for non-destructive device-presence probes */
class II2cDeviceProbe
{
public:
  virtual ~II2cDeviceProbe() = default;

  /** \brief Probe a 7-bit address through the requested adapter */
  virtual I2cProbeResult Probe(const std::string& device_path, int device_address) = 0;
};
} // namespace OASIS::PowerMeter
