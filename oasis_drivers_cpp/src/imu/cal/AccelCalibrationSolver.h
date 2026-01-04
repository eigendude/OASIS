/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "imu/ImuTypes.h"
#include "imu/io/ImuCalibrationFile.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace OASIS::IMU
{
/*!\brief Accel calibration solver for raw samples*/
class AccelCalibrationSolver
{
public:
  struct Result
  {
    /*!
     * \brief Estimated accelerometer bias
     *
     * Units: m/s^2
     */
    Vec3 accel_bias_mps2{0.0, 0.0, 0.0};

    /*!
     * \brief Estimated accelerometer correction matrix
     *
     * Units: unitless
     * Layout: row-major 3x3
     */
    Mat3 accel_A{{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}};

    /*!
     * \brief Covariance of [b_a, vec(A)] parameters
     *
     * Units:
     * - b_a terms: (m/s^2)^2
     * - A terms:   unitless^2
     *
     * Params: [bx, by, bz, a00, a01, a02, a10, a11, a12, a20, a21, a22]
     * Layout: row-major 12x12
     */
    std::array<std::array<double, 12>, 12> accel_param_cov{};

    /*!
     * \brief Fit-quality metric
     *
     * Units: m/s^2
     */
    double rms_residual_mps2{0.0};

    /*!
     * \brief Raw accelerometer ellipsoid fit
     */
    AccelEllipsoid ellipsoid{};

    /*!
     * \brief Number of samples used in the fit
     *
     * Units: samples
     */
    std::uint32_t sample_count{0};
  };

  void Reset();
  void AddSample(const Vec3& accel_mps2);
  std::size_t SampleCount() const;

  bool Solve(double gravity_mps2, Result& out) const;

private:
  std::vector<Vec3> m_samples;
};
} // namespace OASIS::IMU
