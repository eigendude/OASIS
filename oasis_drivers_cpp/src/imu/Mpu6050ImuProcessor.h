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

#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/time.hpp>

namespace OASIS::IMU
{
class Mpu6050ImuProcessor
{
public:
  enum class CalibrationState
  {
    kUncalibrated,
    kGravityCollect,
    kGravitySolved,
    kYawPulseCollect,
    kCalibrated
  };

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
  double GetAccelMagBaseline() const;
  bool HasAccelMagBaseline() const;
  double GetGyroEma() const;
  double GetAccelDevEma() const;
  bool IsStationary() const;
  double GetStillTime() const;
  double GetMoveTime() const;
  bool IsCalibrated() const;
  CalibrationState GetCalibrationState() const;
  bool HasMountingQuaternion() const;
  geometry_msgs::msg::Quaternion GetMountingQuaternionMsg() const;
  std::array<double, 3> GetGravityHat() const;
  std::array<double, 2> GetForwardHatIntermediate() const;
  double GetYawRadians() const;
  double GetYawConfidence() const;

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
  bool m_prevAxisLocked = false;
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
  int m_prevSignedAxisSign = 1;

  CalibrationState m_calState = CalibrationState::kUncalibrated;
  double m_gravityCollectTime = 0.0;
  int m_gravitySamples = 0;
  std::array<double, 3> m_gravitySum{{0.0, 0.0, 0.0}};
  std::array<double, 3> m_gravityHatSolved{{0.0, 0.0, 1.0}};
  std::array<double, 4> m_qGravity{{1.0, 0.0, 0.0, 0.0}};

  bool m_collectingYaw = false;
  std::array<double, 2> m_forwardAccum{{0.0, 0.0}};
  int m_forwardSamples = 0;
  double m_forwardDynamicEma = 0.0;
  double m_pulseTime = 0.0;
  double m_timeSinceSignFlip = 0.0;
  int m_yawTotalSeen = 0;
  int m_yawAccepted = 0;
  int m_yawSkipLowDyn = 0;
  int m_yawSkipNoSign = 0;
  int m_yawSkipSignGuard = 0;
  double m_sumHorizMag = 0.0;
  std::array<double, 2> m_forwardHatIntermediate{{1.0, 0.0}};
  double m_yawRadians = 0.0;
  double m_yawConfidence = 0.0;
  std::array<double, 4> m_qMounting{{1.0, 0.0, 0.0, 0.0}};
};
} // namespace OASIS::IMU
