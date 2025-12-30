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

/**
 * @brief Online accelerometer bias/scale estimator using stationary magnitude residuals.
 *
 * The calibrator watches for stationary periods and nudges a diagonal
 * bias/scale model so the corrected acceleration magnitude matches gravity.
 *
 * Phase 1 ("precal") detects stationarity using gyro + accel first differences,
 * avoids the |a|≈g chicken-and-egg before bias/scale are learned. Phase 2
 * ("strict") activates once uniform scale is initialized and requires the
 * corrected accel magnitude to be near gravity.
 */
class AccelCalibrator
{
public:
  /**
   * @brief Diagnostics snapshot emitted by Update().
   */
  struct Diagnostics
  {
    /// Estimated bias per axis (m/s^2). Applied additively to raw accel.
    std::array<double, 3> bias_mps2{0.0, 0.0, 0.0};
    /// Estimated scale per axis (unitless). Applied multiplicatively.
    std::array<double, 3> scale{1.0, 1.0, 1.0};
    /// True when the active phase's stationary predicate is met.
    bool stationary{false};
    /// True when gyro + accel jerk meet pre-calibration stationary criteria.
    bool stationary_precal{false};
    /// True when accel magnitude and gyro rate meet strict stationary criteria.
    bool stationary_strict{false};
    /// True when dwell is using the post-calibration (strict |a|≈g) gate.
    bool stationary_phase2{false};
    /// True when strict stationary conditions have been held for dwell time.
    bool stationary_confirmed{false};
    /// Accumulated stationary dwell time (s).
    double stationary_dwell_seconds{0.0};
    /// Stationary dwell target (s) required to confirm.
    double stationary_dwell_target_seconds{0.0};

    /// EWMA mean gyro magnitude (rad/s).
    double omega_mean{0.0};
    /// EWMA 1-sigma gyro magnitude noise estimate (rad/s).
    double omega_sigma{0.0};
    /// True when gyro noise statistics have been initialized.
    bool omega_stats_inited{false};
    /// EWMA mean accel delta magnitude (m/s^2).
    double delta_a_mean{0.0};
    /// EWMA 1-sigma accel delta noise estimate (m/s^2).
    double delta_a_sigma{0.0};
    /// True when accel delta noise statistics have been initialized.
    bool delta_a_stats_inited{false};
    /// True when boot-time noise calibration is actively collecting samples.
    bool noise_calib_active{false};
    /// True when boot-time noise calibration has completed.
    bool noise_calib_done{false};
    /// Stationarity score from normalized gyro + accel delta residuals.
    double stationarity_score{0.0};

    /// Coverage flags: true if gravity has pointed mostly in +axis (unit > +0.75).
    std::array<bool, 3> pos_seen{false, false, false};
    /// Coverage flags: true if gravity has pointed mostly in -axis (unit < -0.75).
    std::array<bool, 3> neg_seen{false, false, false};
  };

  AccelCalibrator();

  /**
   * @brief Update the estimator with a new IMU sample.
   *
   * @param accel_mps2 Linear acceleration (m/s^2) in sensor frame.
   * @param gyro_rads Angular velocity (rad/s) in sensor frame.
   * @param dt_seconds Sample interval (s). Non-positive values fall back to 0.02 s.
   * @return Diagnostic snapshot of the current estimator state.
   */
  Diagnostics Update(const std::array<double, 3>& accel_mps2,
                     const std::array<double, 3>& gyro_rads,
                     double dt_seconds);

  /// Latest bias estimate (m/s^2).
  const std::array<double, 3>& GetBias() const;
  /// Latest scale estimate (unitless).
  const std::array<double, 3>& GetScale() const;

  /**
   * @brief Reset uniform scale initialization state.
   *
   * Clears the stationary mean accumulator and allows the uniform scale
   * initialization to run again with new input scaling.
   */
  void ResetUniformScaleInitialization();

private:
  struct RunningMean
  {
    std::size_t n{0};
    double mean{0.0};

    void Add(double x);
    bool Ready(std::size_t min_n) const;
  };

  struct OnlineStats
  {
    double mu{0.0};
    double var{0.0};
    std::size_t n{0};
    bool inited{false};

    void UpdateWinsor(double x,
                      double alpha,
                      double sigma_floor,
                      double sigma_cap,
                      double k_clip);
    double Sigma(double sigma_floor) const;
    double Z(double x, double sigma_floor) const;
    bool Ready(std::size_t min_n) const;
  };

  static double Norm3(const std::array<double, 3>& v);
  bool IsStrictStationary(const std::array<double, 3>& accel_mps2,
                          const std::array<double, 3>& gyro_rads) const;
  bool UpdateStationaryDwell(bool stationary_candidate, bool use_leaky, double dt_seconds);

  // State
  std::array<double, 3> m_bias{0.0, 0.0, 0.0};
  std::array<double, 3> m_scale{1.0, 1.0, 1.0};

  bool m_uniform_scale_initialized{false};
  RunningMean m_stationary_norm_mean;
  double m_stationary_dwell_seconds{0.0};
  bool m_stationary_confirmed{false};
  std::array<double, 3> m_prev_accel_raw_mps2{0.0, 0.0, 0.0};
  bool m_have_prev_accel{false};
  OnlineStats m_omega_stats;
  OnlineStats m_delta_a_stats;
  bool m_stationary_gate_active{false};
  double m_noise_calib_elapsed{0.0};

  std::array<bool, 3> m_pos_seen{false, false, false};
  std::array<bool, 3> m_neg_seen{false, false, false};
};

} // namespace OASIS::IMU
