/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/ImuOrientationUtils.h"

#include "imu/ImuMath.h"

#include <algorithm>

namespace OASIS::IMU::ImuOrientationUtils
{
Math::Quaternion TiltQuaternionFromUp(const std::array<double, 3>& u_hat, bool u_hat_valid)
{
  if (!u_hat_valid)
    return Math::Quaternion{};

  const std::array<double, 3> u_norm = Math::Normalize(u_hat);
  if (Math::Norm(u_norm) <= 0.0)
    return Math::Quaternion{};

  const std::array<double, 3> sensor_x{1.0, 0.0, 0.0};
  const std::array<double, 3> sensor_y{0.0, 1.0, 0.0};

  std::array<double, 3> x_proj =
      Math::Subtract(sensor_x, Math::Scale(u_norm, Math::Dot(sensor_x, u_norm)));
  if (Math::Norm(x_proj) <= 1e-6)
    x_proj = Math::Subtract(sensor_y, Math::Scale(u_norm, Math::Dot(sensor_y, u_norm)));

  const std::array<double, 3> x_world_in_sensor = Math::Normalize(x_proj);
  const std::array<double, 3> y_world_in_sensor =
      Math::Normalize(Math::Cross(u_norm, x_world_in_sensor));

  return Math::FromWorldAxesInSensor(x_world_in_sensor, y_world_in_sensor, u_norm);
}
} // namespace OASIS::IMU::ImuOrientationUtils
