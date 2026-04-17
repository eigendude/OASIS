################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# Predicted IMU orientation product for low-latency / VR-style consumers.
#
# This message is intentionally separate from sensor_msgs/Imu because the
# orientation field is predicted forward by prediction_horizon_sec, while the
# angular velocity and linear acceleration fields remain present-time samples
# associated with the source frame unless otherwise noted by the publisher.
#
# The companion topic `imu_predicted` may publish the same predicted orientation
# in a sensor_msgs/Imu for compatibility with existing consumers, but this
# message is the authoritative place for prediction metadata.
#

std_msgs/Header header

#
# True when the prediction fields are populated and safe to consume.
#
bool valid

#
# Source used to form the prediction.
#
# Suggested values:
# - "gyro_rotation_vector"
# - "gyro_rotation_vector_prediction"
# - "rotation_vector_plus_host_prediction"
#
string source

#
# Prediction horizon applied to orientation.
#
# Units: seconds
# Meaning: orientation estimates attitude at
#          header.stamp + prediction_horizon_sec
#
float64 prediction_horizon_sec

#
# Orientation estimate predicted forward by prediction_horizon_sec.
#
# This attitude estimate corresponds to header.stamp + prediction_horizon_sec.
#
geometry_msgs/Quaternion orientation

#
# Full 3x3 covariance of the predicted orientation error, expressed as a local
# 3-parameter small-angle covariance.
#
# Units: rad^2
# Layout: 3x3 row-major
#
# Interpretation:
# - This is not quaternion element covariance.
# - This is covariance of a local 3-parameter rotation error.
# - Full covariance is allowed.
# - Off-diagonals are currently zero by driver policy unless a better model is
#   introduced later.
#
float64[9] orientation_covariance

#
# SH-2 / driver accuracy bucket associated with the prediction source.
#
# Suggested values:
# - 0: unreliable
# - 1: low
# - 2: medium
# - 3: high
#
uint8 accuracy_status

#
# True when orientation_covariance includes an explicit prediction model rather
# than only the present-time covariance estimate.
#
bool covariance_is_prediction_model_based

#
# Diagnostic best-case orientation sigma used by the driver.
#
# Units: rad
#
float64 sigma_noise_rad

#
# Diagnostic nominal orientation sigma used by the driver.
#
# Units: rad
#
float64 sigma_rms_rad

#
# Diagnostic conservative orientation bound used by the driver.
#
# Units: rad
#
float64 sigma_bound_rad

#
# Present-time angular velocity sample associated with the predicted orientation.
#
# This field is not itself predicted forward by prediction_horizon_sec.
# It remains a present-time sample at header.stamp.
#
# Units: rad/s
#
geometry_msgs/Vector3 angular_velocity

#
# Full 3x3 covariance of present-time angular velocity.
#
# Units: (rad/s)^2
# Layout: 3x3 row-major
#
float64[9] angular_velocity_covariance

#
# Present-time linear acceleration sample associated with the predicted
# orientation.
#
# This field is not itself predicted forward by prediction_horizon_sec.
# It remains a present-time sample at header.stamp.
#
# Units: m/s^2
#
geometry_msgs/Vector3 linear_acceleration

#
# Full 3x3 covariance of present-time linear acceleration.
#
# Units: (m/s^2)^2
# Layout: 3x3 row-major
#
float64[9] linear_acceleration_covariance
