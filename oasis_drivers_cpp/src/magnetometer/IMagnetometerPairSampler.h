/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <string>

#include <Eigen/Core>

namespace OASIS
{
namespace Magnetometer
{

class IMagnetometerPairSampler
{
public:
  virtual ~IMagnetometerPairSampler() = default;

  // Meaning: acquire back-to-back SET and RESET samples, returning Tesla values
  virtual bool SamplePair(Eigen::Vector3d& set_t, Eigen::Vector3d& reset_t) = 0;

  // Meaning: optional human-readable diagnostics for the last failure
  virtual std::string GetLastError() const = 0;
};

} // namespace Magnetometer
} // namespace OASIS
