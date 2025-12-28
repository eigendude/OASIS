/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <array>
#include <optional>

#include <rclcpp/time.hpp>

namespace OASIS::IMU
{
class Mpu6050ImuProcessor
{
public:
  struct ImuSample
  {
    rclcpp::Time stamp;
    std::array<double, 3> accel;
    std::array<double, 3> gyro;
    double duty_cycle = 0.0;
  };

  struct ImuOutput
  {
    std::array<double, 3> linear_acceleration;
    std::array<double, 3> angular_velocity;
    std::array<double, 2> u_hat2d;
    std::array<double, 2> u_signed2d;
    bool axis_locked = false;
    int cmd_sign = 0;
    int signed_sign = 1;
    bool stationary = false;
  };

  Mpu6050ImuProcessor();
  ~Mpu6050ImuProcessor();

  void SetAccelScale(double accelScale);
  void SetGyroScale(double gyroScale);
  double GetAccelScale() const;
  double GetGyroScale() const;

  std::optional<ImuOutput> Process(const ImuSample& sample);

private:
  double m_accelScale = 0.0;
  double m_gyroScale = 0.0;

  bool m_hasTimestamp = false;
  rclcpp::Time m_lastStamp;

  bool m_hasGravity = false;
  std::array<double, 3> m_gravityHat{{0.0, 0.0, 1.0}};

  double m_gMagEma = 0.0;
  bool m_hasGMag = false;

  double m_gyroEma = 0.0;
  double m_accelDevEma = 0.0;
  double m_stillTime = 0.0;
  double m_moveTime = 0.0;
  bool m_stationary = true;

  double m_cov00 = 0.0;
  double m_cov01 = 0.0;
  double m_cov11 = 0.0;
  std::array<double, 2> m_axisHat{{1.0, 0.0}};
  bool m_axisLocked = false;
  double m_lockAccumTime = 0.0;
  int m_covSamples = 0;
  bool m_axisResetArmed = true;

  double m_cmdFiltered = 0.0;
  int m_cmdSign = 0;
  double m_reversalGraceTime = 0.0;
  double m_signConfirmTime = 0.0;
  int m_confirmTarget = 0;

  double m_imuEvidence = 0.0;
  int m_signedAxisSign = 1;
};
} // namespace OASIS::IMU
