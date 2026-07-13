/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "I2cDeviceProbe.hpp"

#include <cstdint>

namespace OASIS::PowerMeter
{
// ACS37800-DS Rev. 4, Register 0x20 is a safe read-only volatile register
constexpr std::uint8_t ACS37800_PROBE_REGISTER = 0x20;

/** \brief Classify errno from an attempted I2C_RDWR device transaction */
I2cProbeStatus ClassifyI2cTransferError(int error_number);

/** \brief Linux I2C_RDWR implementation of the ACS37800 presence probe */
class LinuxI2cDeviceProbe : public II2cDeviceProbe
{
public:
  I2cProbeResult Probe(const std::string& device_path, int device_address) override;
};
} // namespace OASIS::PowerMeter
