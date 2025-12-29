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

namespace OASIS::IMU
{
/**
 * @brief Learn an unsigned forward axis from straight acceleration bursts.
 */
class ForwardAxisLearner
{
public:
  /**
   * @brief Learning state.
   */
  enum class State
  {
    /// Waiting for a straight acceleration burst.
    ARMED,
    /// Accumulating burst samples.
    IN_BURST,
    /// Learned first signed axis, waiting for opposite burst.
    HAVE_FIRST,
    /// Learned signed forward axis.
    LOCKED
  };

  /**
   * @brief Diagnostics snapshot emitted by Update().
   */
  struct Diagnostics
  {
    /// Current state machine state.
    State state{State::ARMED};
    /// Yaw rate about up axis (rad/s).
    double yaw_rate_rads{0.0};
    /// Horizontal acceleration magnitude (m/s^2).
    double a_h_mag_mps2{0.0};
    /// Integrated horizontal impulse (m/s).
    double impulse_mps{0.0};
    /// Integrated yaw during burst (rad).
    double yaw_accum_rad{0.0};
    /// Burst duration (s).
    double burst_time_s{0.0};
    /// Number of accepted bursts.
    std::size_t burst_count{0};
    /// Consistency dot between latest candidate and mean (unitless).
    double consistency_dot{0.0};
    /// True if the unsigned forward axis is locked.
    bool locked{false};
  };

  ForwardAxisLearner();

  Diagnostics Update(const std::array<double, 3>& accel_mps2,
                     const std::array<double, 3>& gyro_rads,
                     double dt_seconds,
                     const std::array<double, 3>& u_hat,
                     bool u_hat_valid);

  /// True when the signed forward axis has been locked.
  bool IsLocked() const;
  /// Learned unsigned forward axis (unit vector in sensor frame).
  const std::array<double, 3>& GetForwardAxisUnsigned() const;
  /// Orthonormal forward axis re-orthogonalized with up (unit vector).
  const std::array<double, 3>& GetForwardAxisOrtho() const;
  /// Current state machine state.
  State GetState() const;

private:
  void StartBurst();
  void ResetBurst();
  void EvaluateBurst(const std::array<double, 3>& u_hat);

  State m_state{State::ARMED};
  std::size_t m_start_counter{0};
  std::size_t m_end_counter{0};
  std::size_t m_yaw_over_counter{0};

  std::array<double, 3> m_sum_vec{0.0, 0.0, 0.0};
  double m_impulse{0.0};
  double m_yaw_accum{0.0};
  double m_t_burst{0.0};
  double m_a_h_peak{0.0};
  double m_yaw_rate_peak{0.0};

  std::size_t m_burst_count{0};
  std::array<double, 3> m_first_candidate{1.0, 0.0, 0.0};
  std::array<double, 3> m_last_candidate{1.0, 0.0, 0.0};
  double m_consistency_dot{0.0};

  std::array<double, 3> m_f_hat_unsigned{1.0, 0.0, 0.0};
  std::array<double, 3> m_f_hat{1.0, 0.0, 0.0};
};
} // namespace OASIS::IMU
