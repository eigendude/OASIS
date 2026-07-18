/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

namespace OASIS::Visualization
{
/** Return whether a rotation nonlinearity produces a monotonic phase warp */
bool IsValidRotationNonlinearity(double nonlinearity);

/**
 * Warp a normalized linear rotation phase into an angle
 *
 * \param phase Linear revolution phase in the range [0, 1]
 * \param nonlinearity Speed variation in the range [0, 1)
 * \return Warped rotation angle in radians
 * \throws std::invalid_argument if either argument is outside its range or
 *         non-finite
 */
double CalculateRotationAngle(double phase, double nonlinearity);

/**
 * Calculate angular velocity relative to the average revolution velocity
 *
 * \param phase Linear revolution phase in the range [0, 1]
 * \param nonlinearity Speed variation in the range [0, 1)
 * \return Relative angular velocity, from 1 - nonlinearity to
 *         1 + nonlinearity
 * \throws std::invalid_argument if either argument is outside its range or
 *         non-finite
 */
double CalculateRelativeAngularVelocity(double phase, double nonlinearity);
} // namespace OASIS::Visualization
