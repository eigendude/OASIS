/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "imu/ImuTypes.h"

#include <cstdint>

namespace OASIS::IMU::BNO086
{
/*!
 * \brief Source selected by the orientation covariance policy module
 */
enum class OrientationCovarianceSource : std::uint8_t
{
  RotationVectorAccuracyEstimate,
  AccuracyBucketFallback,
};

/*!
 * \brief Driver-owned orientation covariance decision for one SH-2 sample
 *
 * The policy module owns the driver-boundary heuristic that turns SH-2
 * Rotation Vector accuracy signals into the published orientation covariance.
 * Rotation Vector estimated accuracy is preferred when present and sane;
 * otherwise a tighter fallback bucket table is used. The node consumes this
 * result and reports which source was selected, while downstream AHRS only
 * preserves and rotates the published covariance.
 */
struct OrientationCovariancePolicyResult
{
  /*!
   * \brief Parsed SH-2 accuracy bucket used for status and fallback
   *
   * Units: enum code in range [0, 3]
   */
  std::uint8_t accuracy_bucket{0};

  /*!
   * \brief Raw SH-2 estimated accuracy field from Rotation Vector
   *
   * Units: SH-2 fixed-point Q12 radians
   */
  std::int16_t raw_accuracy_estimate_q12{0};

  /*!
   * \brief True when the Rotation Vector report carried a usable estimate
   *
   * Units: boolean
   */
  bool has_accuracy_estimate{false};

  /*!
   * \brief Decoded SH-2 estimated accuracy in radians when available
   *
   * Units: rad
   */
  double accuracy_estimate_rad{0.0};

  /*!
   * \brief Final published 1-sigma orientation uncertainty per axis
   *
   * Units: rad
   */
  double sigma_rad{0.0};

  /*!
   * \brief Final published orientation covariance
   *
   * Units: rad^2
   * Layout: row-major 3x3 diagonal
   */
  Mat3 covariance_rad2{};

  /*!
   * \brief Policy source used to choose \ref sigma_rad
   */
  OrientationCovarianceSource source{OrientationCovarianceSource::AccuracyBucketFallback};
};

OrientationCovariancePolicyResult ResolveOrientationCovariancePolicy(
    std::uint8_t accuracy_bucket, std::int16_t raw_accuracy_estimate_q12);

double OrientationAccuracyBucketToSigmaRad(std::uint8_t accuracy_bucket);

const char* OrientationCovarianceSourceName(OrientationCovarianceSource source);
} // namespace OASIS::IMU::BNO086
