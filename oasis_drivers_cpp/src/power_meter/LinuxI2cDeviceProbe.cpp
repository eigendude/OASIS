/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "LinuxI2cDeviceProbe.hpp"

#include "LinuxI2cRegisterDevice.hpp"

#include <cerrno>
#include <cstring>
#include <string>
#include <system_error>

using OASIS::PowerMeter::I2cProbeResult;
using OASIS::PowerMeter::I2cProbeStatus;

namespace
{
I2cProbeResult ErrorResult(const std::string& operation, int error_number)
{
  return {I2cProbeStatus::Error,
          operation + " failed: " + std::string(std::strerror(error_number))};
}
} // namespace

I2cProbeStatus OASIS::PowerMeter::ClassifyI2cTransferError(int error_number)
{
  switch (error_number)
  {
    case ENXIO:
    case EREMOTEIO:
    case EIO:
      return I2cProbeStatus::NotPresent;
    default:
      return I2cProbeStatus::Error;
  }
}

I2cProbeResult OASIS::PowerMeter::LinuxI2cDeviceProbe::Probe(const std::string& device_path,
                                                             int device_address)
{
  if (device_address < 0x03 || device_address > 0x77)
    return {I2cProbeStatus::Error, "device address must be in [0x03, 0x77]"};

  try
  {
    LinuxI2cRegisterDevice device(device_path, device_address);
    (void)device.ReadRegister(ACS37800_PROBE_REGISTER);
  }
  catch (const std::system_error& error)
  {
    if (ClassifyI2cTransferError(error.code().value()) == I2cProbeStatus::NotPresent)
      return {I2cProbeStatus::NotPresent, {}};
    return ErrorResult("ACS37800 probe", error.code().value());
  }
  catch (const std::exception& error)
  {
    return {I2cProbeStatus::Error, error.what()};
  }

  return {I2cProbeStatus::Present, {}};
}
