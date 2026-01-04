/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "magnetometer/Mmc5983maPairSampler.h"

namespace OASIS
{
namespace Magnetometer
{

Mmc5983maPairSampler::Mmc5983maPairSampler(Mmc5983maDevice& device) : m_device(device)
{
}

bool Mmc5983maPairSampler::SamplePair(Eigen::Vector3d& set_t, Eigen::Vector3d& reset_t)
{
  m_lastError.clear();

  if (!m_device.TakeMeasurement(MeasurementMode::Set, set_t))
  {
    m_lastError = "Failed to read MMC5983MA SET measurement";
    return false;
  }

  if (!m_device.TakeMeasurement(MeasurementMode::Reset, reset_t))
  {
    m_lastError = "Failed to read MMC5983MA RESET measurement";
    return false;
  }

  return true;
}

std::string Mmc5983maPairSampler::GetLastError() const
{
  return m_lastError;
}

} // namespace Magnetometer
} // namespace OASIS
