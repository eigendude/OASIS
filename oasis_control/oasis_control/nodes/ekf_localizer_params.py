################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""
Centralized EKF localizer ROS parameter names and defaults
"""

# Duration of state history buffer used for late sensor fusion, nanoseconds
PARAM_T_BUFFER_NS: str = "t_buffer_ns"

# Default duration of state history buffer used for late sensor fusion, nanoseconds
DEFAULT_T_BUFFER_NS: int = 2_000_000_000

# Allowed future wall-time tolerance for timestamps, nanoseconds
PARAM_EPS_WALL_FUTURE_NS: str = "epsilon_wall_future_ns"

# Default future wall-time tolerance for timestamps, nanoseconds
DEFAULT_EPS_WALL_FUTURE_NS: int = 50_000_000

# Maximum acceptable wall-clock jump when timestamping, nanoseconds
PARAM_DT_CLOCK_JUMP_MAX_NS: str = "dt_clock_jump_max_ns"

# Default maximum acceptable wall-clock jump when timestamping, nanoseconds
DEFAULT_DT_CLOCK_JUMP_MAX_NS: int = 500_000_000

# Maximum IMU sample delta time before rejecting updates, nanoseconds
PARAM_DT_IMU_MAX_NS: str = "dt_imu_max_ns"

# Default maximum IMU sample delta time before rejecting updates, nanoseconds
DEFAULT_DT_IMU_MAX_NS: int = 50_000_000

# Position measurement variance for EKF updates, meters^2
PARAM_POS_VAR: str = "pos_var"

# Default position measurement variance for EKF updates, meters^2
DEFAULT_POS_VAR: float = 1.0

# Velocity measurement variance for EKF updates, meters^2/seconds^2
PARAM_VEL_VAR: str = "vel_var"

# Default velocity measurement variance for EKF updates, meters^2/seconds^2
DEFAULT_VEL_VAR: float = 1.0

# Orientation measurement variance for EKF updates, radians^2
PARAM_ANG_VAR: str = "ang_var"

# Default orientation measurement variance for EKF updates, radians^2
DEFAULT_ANG_VAR: float = 0.25

# Accelerometer process noise variance, meters^2/seconds^4
PARAM_ACCEL_NOISE_VAR: str = "accel_noise_var"

# Default accelerometer process noise variance, meters^2/seconds^4
DEFAULT_ACCEL_NOISE_VAR: float = 1.0

# Gyroscope process noise variance, radians^2/seconds^2
PARAM_GYRO_NOISE_VAR: str = "gyro_noise_var"

# Default gyroscope process noise variance, radians^2/seconds^2
DEFAULT_GYRO_NOISE_VAR: float = 0.01

# Gravity magnitude used in the IMU model, meters/seconds^2
PARAM_GRAVITY_MPS2: str = "gravity_mps2"

# Default gravity magnitude used in the IMU model, meters/seconds^2
DEFAULT_GRAVITY_MPS2: float = 9.80665

# Maximum integration step for EKF propagation, nanoseconds
PARAM_MAX_DT_NS: str = "max_dt_ns"

# Default maximum integration step for EKF propagation, nanoseconds
DEFAULT_MAX_DT_NS: int = 50_000_000

# Interval between state checkpoints in the buffer, nanoseconds
PARAM_CHECKPOINT_INTERVAL_NS: str = "checkpoint_interval_ns"

# Default interval between state checkpoints in the buffer, nanoseconds
DEFAULT_CHECKPOINT_INTERVAL_NS: int = 500_000_000

# AprilTag position measurement variance, meters^2
PARAM_APRILTAG_POS_VAR: str = "apriltag_pos_var"

# Default AprilTag position measurement variance, meters^2
DEFAULT_APRILTAG_POS_VAR: float = 0.25

# AprilTag yaw measurement variance, radians^2
PARAM_APRILTAG_YAW_VAR: str = "apriltag_yaw_var"

# Default AprilTag yaw measurement variance, radians^2
DEFAULT_APRILTAG_YAW_VAR: float = 0.1

# Squared gating threshold for AprilTag Mahalanobis distance, unitless
PARAM_APRILTAG_GATE_D2: str = "apriltag_gate_d2"

# Default squared gating threshold for AprilTag Mahalanobis distance, unitless
DEFAULT_APRILTAG_GATE_D2: float = 9.49

# RMS reprojection gate for AprilTag corner residuals, pixels
PARAM_APRILTAG_REPROJ_RMS_GATE_PX: str = "apriltag_reproj_rms_gate_px"

# Default RMS reprojection gate for AprilTag corner residuals, pixels
DEFAULT_APRILTAG_REPROJ_RMS_GATE_PX: float = 5.0

# Prior translation sigma for tag landmarks, meters
PARAM_TAG_LANDMARK_PRIOR_SIGMA_T_M: str = "tag_landmark_prior_sigma_t_m"

# Default prior translation sigma for tag landmarks, meters
DEFAULT_TAG_LANDMARK_PRIOR_SIGMA_T_M: float = 10.0

# Prior rotation sigma for tag landmarks, radians
PARAM_TAG_LANDMARK_PRIOR_SIGMA_ROT_RAD: str = "tag_landmark_prior_sigma_rot_rad"

# Default prior rotation sigma for tag landmarks, radians
DEFAULT_TAG_LANDMARK_PRIOR_SIGMA_ROT_RAD: float = 3.141592653589793

# Prior translation sigma for extrinsics, meters
PARAM_EXTRINSIC_PRIOR_SIGMA_T_M: str = "extrinsic_prior_sigma_t_m"

# Default prior translation sigma for extrinsics, meters
DEFAULT_EXTRINSIC_PRIOR_SIGMA_T_M: float = 1.0

# Prior rotation sigma for extrinsics, radians
PARAM_EXTRINSIC_PRIOR_SIGMA_ROT_RAD: str = "extrinsic_prior_sigma_rot_rad"

# Default prior rotation sigma for extrinsics, radians
DEFAULT_EXTRINSIC_PRIOR_SIGMA_ROT_RAD: float = 3.141592653589793
