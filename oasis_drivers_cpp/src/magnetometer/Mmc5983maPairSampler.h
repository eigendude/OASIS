/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "magnetometer/IMagnetometerPairSampler.h"
#include "magnetometer/Mmc5983maDevice.h"

#include <string>

namespace OASIS
{
namespace Magnetometer
{

class Mmc5983maPairSampler : public IMagnetometerPairSampler
{
public:
  explicit Mmc5983maPairSampler(Mmc5983maDevice& device);

  bool SamplePair(Eigen::Vector3d& set_t, Eigen::Vector3d& reset_t) override;

  std::string GetLastError() const override;

private:
  Mmc5983maDevice& m_device;
  std::string m_lastError;
};

} // namespace Magnetometer
} // namespace OASIS
