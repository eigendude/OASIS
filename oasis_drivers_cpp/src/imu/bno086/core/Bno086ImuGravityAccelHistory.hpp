/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "imu/ImuTypes.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace OASIS::IMU::BNO086
{
/*!
 * \brief Gravity-included calibrated acceleration sample for imu_gravity
 */
struct Bno086ImuGravityAccelSample
{
  /*!
   * \brief True when this entry contains a real sensor sample
   */
  bool has_sample{false};

  /*!
   * \brief Sample timestamp on the caller timeline
   *
   * Units: ns
   */
  int64_t stamp_ns{0};

  /*!
   * \brief Calibrated acceleration including gravity
   *
   * Units: m/s^2
   */
  Vec3 accel_mps2{0.0, 0.0, 0.0};

  /*!
   * \brief Acceleration covariance when available
   *
   * Units: (m/s^2)^2
   */
  Mat3 covariance_mps2_2{};

  /*!
   * \brief True when \ref covariance_mps2_2 is populated
   */
  bool has_covariance{false};

  /*!
   * \brief Source BNO086 report sequence value
   *
   * Units: unsigned 8-bit counter ticks
   */
  std::uint8_t sequence{0};

  /*!
   * \brief Source SH-2 accuracy bucket
   *
   * Units: enum code in range [0, 3]
   */
  std::uint8_t accuracy{0};
};

/*!
 * \brief Fixed-size history for selecting imu_gravity acceleration samples
 */
class Bno086ImuGravityAccelHistory
{
public:
  void Push(const Bno086ImuGravityAccelSample& sample);

  std::optional<Bno086ImuGravityAccelSample> SelectAtOrBefore(int64_t anchor_stamp_ns,
                                                              int64_t future_tolerance_ns) const;

  void Reset();

private:
  std::array<Bno086ImuGravityAccelSample, 1024> m_samples{};
  std::size_t m_next{0};
  std::size_t m_count{0};
};
} // namespace OASIS::IMU::BNO086
