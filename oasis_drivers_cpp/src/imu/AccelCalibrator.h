/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace OASIS::IMU
{

class AccelCalibrator
{
public:
  struct Diagnostics
  {
    std::array<double, 3> bias_mps2{0.0, 0.0, 0.0};
    std::array<double, 3> scale{1.0, 1.0, 1.0};
    bool stationary{false};

    // Face detection (when stationary)
    bool face_valid{false};
    std::size_t face_axis{0}; // 0=x,1=y,2=z
    int face_sign{0}; // +1 or -1
    double face_cos{0.0}; // |unit_component| on the dominant axis (0..1)

    // Coverage
    std::array<bool, 3> pos_seen{false, false, false};
    std::array<bool, 3> neg_seen{false, false, false};
  };

  AccelCalibrator();

  // accel_mps2: already converted to m/s^2
  // gyro_rads: already converted to rad/s
  Diagnostics Update(const std::array<double, 3>& accel_mps2,
                     const std::array<double, 3>& gyro_rads,
                     double dt_seconds);

  const std::array<double, 3>& GetBias() const;
  const std::array<double, 3>& GetScale() const;

private:
  struct RunningMean
  {
    std::size_t n{0};
    double mean{0.0};

    void Add(double x);
    bool Ready(std::size_t min_n) const;
  };

  static double Norm3(const std::array<double, 3>& v);

  // State
  bool m_initialized{false};
  std::array<double, 3> m_accel_lp{0.0, 0.0, 0.0};
  std::array<double, 3> m_prev_accel_lp{0.0, 0.0, 0.0};

  std::array<double, 3> m_bias{0.0, 0.0, 0.0};
  std::array<double, 3> m_scale{1.0, 1.0, 1.0};

  bool m_uniform_scale_initialized{false};
  RunningMean m_stationary_norm_mean;

  // Per-axis face means (only the axis component in that face)
  std::array<RunningMean, 3> m_pos_face_mean;
  std::array<RunningMean, 3> m_neg_face_mean;

  std::array<bool, 3> m_pos_seen{false, false, false};
  std::array<bool, 3> m_neg_seen{false, false, false};
};

} // namespace OASIS::IMU
