/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <array>
#include <optional>
#include <string>

namespace OASIS::Localization
{
using Vector3 = std::array<double, 3>;

/*!
 * \brief Configuration values for the IMU-only ZUPT detector
 */
struct ZuptDetectorConfig
{
  /*!
   * \brief Gyro norm threshold in rad/s required to enter stationary mode
   *
   * Expected range is positive and below the moving threshold. The detector
   * compares the angular-velocity norm against this value during quiet dwell.
   */
  double gyro_enter_threshold_rads{0.06};

  /*!
   * \brief Gyro norm threshold in rad/s required to exit stationary mode
   *
   * Expected range is positive and above the enter threshold. A threshold
   * breach contributes raw motion evidence for clearing stationary.
   */
  double gyro_exit_threshold_rads{0.09};

  /*!
   * \brief Linear-acceleration norm threshold in m/s^2 for stationary entry
   *
   * Expected range is positive and below the moving threshold. The detector
   * compares the linear-acceleration norm against this value during quiet
   * dwell.
   */
  double accel_enter_threshold_mps2{0.18};

  /*!
   * \brief Linear-acceleration norm threshold in m/s^2 for stationary exit
   *
   * Expected range is positive and above the enter threshold. A threshold
   * breach contributes raw motion evidence for clearing stationary.
   */
  double accel_exit_threshold_mps2{0.28};

  /*!
   * \brief Minimum quiet dwell time in seconds before stationary assertion
   *
   * Expected range is non-negative. Computed as the current sample timestamp
   * minus the start time of the current enter candidate.
   */
  double min_stationary_sec{0.18};

  /*!
   * \brief Minimum moving dwell time in seconds before stationary clearing
   *
   * Expected range is non-negative. Computed as the current sample timestamp
   * minus the start time of the current exit candidate.
   */
  double min_moving_sec{0.01};

  /*!
   * \brief Stationary linear-velocity sigma in m/s
   *
   * Expected range is positive. The square of this value is published on the
   * linear diagonal of the stationary ZUPT covariance.
   */
  double stationary_linear_velocity_sigma_mps{0.06};

  /*!
   * \brief Stationary angular-velocity sigma in rad/s
   *
   * Expected range is positive. The square of this value is published on the
   * angular diagonal of the stationary ZUPT covariance.
   */
  double stationary_angular_velocity_sigma_rads{0.06};

  /*!
   * \brief Moving linear-velocity variance in (m/s)^2
   *
   * Expected range is positive. Used while moving and as the sanitization
   * fallback for invalid published linear variances.
   */
  double moving_linear_variance_mps2{1.0e6};

  /*!
   * \brief Moving angular-velocity variance in (rad/s)^2
   *
   * Expected range is positive. Used while moving and as the sanitization
   * fallback for invalid published angular variances.
   */
  double moving_angular_variance_rads2{1.0e6};
};

/*!
 * \brief Mutable state for the IMU-only ZUPT detector
 */
struct ZuptDetectorState
{
  /*! \brief True when the platform is currently considered stationary */
  bool stationary{false};

  /*! \brief Timestamp in seconds when the enter candidate began */
  std::optional<double> enter_candidate_start_sec;

  /*! \brief Timestamp in seconds when the exit candidate began */
  std::optional<double> exit_candidate_start_sec;

  /*! \brief Last accepted timestamp in seconds */
  std::optional<double> last_timestamp_sec;

  /*! \brief Last finite angular-velocity norm in rad/s */
  double last_gyro_norm_rads{0.0};

  /*! \brief Last finite linear-acceleration norm in m/s^2 */
  double last_accel_norm_mps2{0.0};

  /*! \brief Current published linear ZUPT variance in (m/s)^2 */
  double current_linear_zupt_variance_mps2{0.0};

  /*! \brief Current published angular ZUPT variance in (rad/s)^2 */
  double current_angular_zupt_variance_rads2{0.0};

  /*! \brief Short reason string for the last detector decision */
  std::string last_reason{"init"};
};

/*!
 * \brief Detector output for a single accepted IMU sample
 */
struct ZuptDecision
{
  /*! \brief True when the platform is considered stationary */
  bool stationary{false};

  /*! \brief Angular-velocity norm in rad/s from the current IMU sample */
  double gyro_norm_rads{0.0};

  /*! \brief Linear-acceleration norm in m/s^2 from the current IMU sample */
  double accel_norm_mps2{0.0};

  /*! \brief Published linear stationary-twist variance in (m/s)^2 */
  double linear_zupt_variance_mps2{0.0};

  /*! \brief Published angular stationary-twist variance in (rad/s)^2 */
  double angular_zupt_variance_rads2{0.0};

  /*! \brief Time in seconds that the enter candidate has been active */
  double enter_dwell_sec{0.0};

  /*! \brief Time in seconds that the exit candidate has been active */
  double exit_dwell_sec{0.0};

  /*! \brief Short reason string for the current detector decision */
  std::string reason{"init"};
};

/*!
 * \brief IMU norm-threshold ZUPT detector with dwell timers
 */
class ZuptDetector
{
public:
  explicit ZuptDetector(ZuptDetectorConfig config);

  std::optional<ZuptDecision> Update(double timestamp_sec,
                                     const Vector3& angular_velocity_rads,
                                     const Vector3& linear_accel_mps2);

  const ZuptDetectorState& State() const { return m_state; }

private:
  void UpdateEnterCandidate(double timestamp_sec, bool enter_candidate);
  void UpdateExitCandidate(double timestamp_sec,
                           std::optional<double> previous_timestamp_sec,
                           bool exit_candidate);
  double StationaryLinearVarianceMps2() const;
  double StationaryAngularVarianceRads2() const;
  ZuptDecision BuildDecision(const std::string& reason) const;

  ZuptDetectorConfig m_config;
  ZuptDetectorState m_state;
};
} // namespace OASIS::Localization
