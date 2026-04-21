# EKF Design Document

This document defines the localization EKF around the current sensor contract.

The EKF fuses:

- `imu` for high-rate motion and attitude information
- `gravity` for gravity-direction consistency and roll/pitch stabilization
- `apriltags` for global pose correction and landmark constraints
- `camera_info` for the AprilTag reprojection model

The old split sensor pipeline is gone. The EKF is defined around the fused IMU
message that already contains a usable quaternion orientation estimate. Gravity
is a first-class required measurement, not an optional addon.

The EKF is the localization-layer component. It owns drift correction,
landmark fusion, and non-identity `world -> odom` behavior, and it is the
component responsible for global correction from AprilTags.

---

## 1. World frame contract

The EKF maintains the standard frames:

- `world`: drift-corrected global frame and the parent of tag landmarks
- `odom`: continuous local frame for downstream motion consumers
- `base_link`: vehicle base frame
- `imu_link`: canonical IMU frame
- `camera_link`: first-class camera frame used by `apriltags` and
  `camera_info`

Symbol aliases used below:

- `{W}` = `world`
- `{O}` = `odom`
- `{B}` = `base_link`
- `{I}` = `imu_link`
- `{C}` = `camera_link`

TF contract:

`world -> odom -> base_link`

`odom -> base_link` must remain continuous.
`world -> odom` may change when global measurements correct drift.

Camera-frame policy:

- `camera_link` is the canonical frame for image-space AprilTag measurements
  and camera intrinsics
- `T_BC` is fixed and maps camera-frame quantities into base-frame quantities
- the current design assumes co-located camera and IMU origins
- the camera may still have a fixed rotational offset relative to `base_link`
- detailed reprojection and camera-measurement policy belongs in
  `EKF_AprilTags.md`

---

## 2. ROS inputs

### 2.1 `imu` — `sensor_msgs/Imu`

Role:

- propagation input for the motion model
- attitude observation whose published covariance now has split semantics:
  gravity-observable roll/pitch covariance plus separately modeled yaw
  variance

Used fields:

- `header.stamp`
- `header.frame_id` as IMU frame `{I}`
- `orientation`, `orientation_covariance`
- `angular_velocity`, `angular_velocity_covariance`
- `linear_acceleration`, `linear_acceleration_covariance`

Requirements:

- quaternion must be finite and non-zero norm
- quaternion is normalized before use
- full covariances are preserved when valid
- IMU vectors are expressed in `{I}`
- `imu.header.frame_id == "imu_link"` by policy

The EKF uses the fixed transform `T_BI` from `imu_link` to `base_link` to map
measurements into base coordinates. The current default source for `T_BI` is
the boot-time mounting calibration contract in `AHRS_Mounting.md`, though the
EKF only assumes that a fixed extrinsic is already available.

For the mounted `ahrs/imu` stream, the published orientation covariance should
be interpreted as an AHRS-owned downstream contract rather than a simple copy
of coarse driver orientation buckets:

- the roll/pitch block reflects gravity-observable attitude uncertainty after
  mounting
- that roll/pitch block is computed by propagating mounted gravity covariance
  through the OASIS roll/pitch mapping
- roll variance and pitch variance may differ
- roll/pitch cross-covariance may be nonzero
- the yaw variance entry is modeled separately by AHRS because gravity does not
  observe heading

### 2.2 `gravity` — `geometry_msgs/AccelWithCovarianceStamped`

Role:

- gravity-direction observation
- roll/pitch stabilization
- observability aid for IMU-to-base mounting consistency

Used fields:

- `header.stamp`
- `header.frame_id`
- `accel.accel.linear`
- `accel.covariance`

Requirements:

- gravity is required
- the vector must be finite and non-zero norm
- `gravity.header.frame_id == "imu_link"` by policy
- `gravity.accel.accel.linear` is expressed in `imu_link`
- the vector points down in the direction of gravitational acceleration
- the magnitude is expected to be near `9.81 m/s^2` at rest
- this is a physical gravity vector, not an "up" vector and not a normalized
  unit vector
- the measurement is not used for propagation
- the measurement is not a source of absolute yaw
- full covariance from the message is preserved

The EKF maps this measurement from `imu_link` into `base_link` using `T_BI`
and applies it as a first-class update.

### 2.3 `apriltags` — `apriltag_msgs/AprilTagDetectionArray`

Role:

- measurement updates for global base pose
- landmark-based global correction using fixed camera extrinsics

Used fields:

- `header.stamp`
- `header.frame_id` as camera frame `{C}`
- `family`
- `id`
- `corners[4]`
- `decision_margin`
- `homography[9]`

Tag identity key:

- `TagKey := (family, id)`

Requirements:

- `apriltags.header.frame_id == "camera_link"` by policy
- detections are interpreted in the canonical camera frame `camera_link`

### 2.4 `camera_info` — `sensor_msgs/CameraInfo`

Role:

- cached intrinsics for AprilTag reprojection

Contract:

- cache the first valid message
- require frame consistency with `apriltags.header.frame_id`
- the camera frame is `camera_link`
- `camera_info.header.frame_id == "camera_link"` by policy
- `apriltags.header.frame_id` and `camera_info.header.frame_id` are both
  expected to be `camera_link`
- reject AprilTag updates until intrinsics are available

---

## 3. Estimated state

The EKF maintains one shared localization state.

Navigation:

- `p_WB ∈ ℝ³`
- `v_WB ∈ ℝ³`
- `q_WB`

Motion nuisance terms:

- `b_g ∈ ℝ³`
- `b_a ∈ ℝ³`

Extrinsics:

- `T_BI`
- `T_BC`

Landmarks:

- `T_WTag(TagKey)` for each initialized tag

Optional environment constants:

- `g_W`

The covariance is full and never diagonalized.

Extrinsic policy:

- `T_BI` is the fixed transform from `imu_link` to `base_link`
- `T_BC` is fixed and maps camera-frame quantities into base-frame quantities
- camera rotation relative to `base_link` may still be nonzero
- these camera extrinsics are fixed by current policy and are not estimated
  online here

---

## 4. Motion and update contract

The EKF propagates on each accepted `imu` sample.

### 4.1 Inputs used for propagation

After mapping the IMU sample into `{B}`:

- angular velocity drives attitude propagation
- linear acceleration drives velocity and position propagation
- the fused quaternion may be used as a direct attitude observation during the
  same update cycle

Gravity is not a propagation input.

### 4.2 Process equations

Navigation kinematics:

- `ṗ_WB = v_WB`
- `v̇_WB = R_BW * (a_B - b_a) + g_W`
- `q̇_WB = 0.5 * Ω(ω_B - b_g) * q_WB`

Bias model:

- `ḃ_g = w_bg`
- `ḃ_a = w_ba`

The process noise covariance is full and carried through standard
linearization/discretization.

---

## 5. Measurement contract

### 5.1 IMU attitude observation

The quaternion in `imu.orientation` is treated as a direct observation of IMU
attitude.

Using the fixed mounting rotation `q_BI`:

- predicted IMU attitude: `q_WI = q_IB ⊗ q_WB`
- residual: minimal 3D tangent error between measured and predicted IMU
  attitude

This update is the main source of short-term fused attitude consistency coming
from the published AHRS quaternion. Its covariance should be interpreted with
the same split semantics described above: gravity-observable roll/pitch block,
separately modeled yaw variance.

### 5.2 Gravity measurement

The gravity message is treated as a first-class IMU-frame gravity measurement
that is transported into the base frame. The shared world-frame gravity
direction remains `g_W = (0, 0, -1)`.

Using the current base attitude estimate:

- predicted base-frame gravity direction: `g_B_pred = R_BW * g_W`
- measured base-frame gravity direction: `g_B_meas = R_BI * g_I_meas`

The innovation is the directional mismatch between `g_B_meas` and `g_B_pred`.
Implementation details may vary, but the contract is:

- gravity is updated as a first-class measurement
- full covariance from `gravity` is used directly after frame transport
- covariance is not diagonalized by convenience
- the measurement improves roll/pitch consistency
- the measurement can help constrain IMU-to-base mounting errors
- the measurement does not provide absolute yaw

At the AHRS layer, the same gravity measurement also drives the published
roll/pitch orientation covariance block by covariance propagation through the
roll/pitch mapping. EKF consumers should therefore not interpret the
`ahrs/imu.orientation_covariance` roll/pitch entries as generic driver bucket
values anymore. They now represent gravity-observable attitude uncertainty,
while the yaw entry remains separately modeled because gravity does not observe
heading.

If `g_W` is modeled as part of the state, the same residual still applies.
Otherwise `g_W` is treated as a fixed environment constant.

### 5.3 AprilTag localization update

Each accepted detection contributes an image-space reprojection constraint.

At the contract level:

- current camera pose: `T_WC = T_WB * inverse(T_BC)`
- `T_BC` is fixed from `camera_link` to `base_link`
- predicted tag pose in camera frame depends on `T_WTag(TagKey)`
- project the known tag corners using `camera_info`
- innovation is pixel reprojection error

Detections from one message are applied sequentially in deterministic order by
`(family, id)`. The focused landmark and reprojection policy lives in
`EKF_AprilTags.md`.

---

## 6. Time handling

Definitions:

- `t_meas_ns`: authoritative event time from message headers
- `t_filter_ns`: current EKF frontier time

The EKF remains fixed-lag capable because AprilTag updates may arrive late
relative to the IMU stream.

Buffer policy:

- retain time-ordered nodes over a configured lag window
- store propagated state, covariance, and attached measurements
- replay deterministically after out-of-order landmark updates

Per-node order:

1. propagate from previous node using stored IMU samples
2. apply IMU attitude update if present
3. apply gravity update if present
4. apply AprilTag updates in sorted tag order

---

## 7. Outputs

### 7.1 TF

- `world -> odom`
- `odom -> base_link`

### 7.2 State outputs

Recommended topics:

- `ekf/odom` as `nav_msgs/Odometry`
- `ekf/state` for full mean state and covariance
- `ekf/updates/apriltag` for per-update diagnostics
- `ekf/diag` for buffer, replay, and reject counters

Per-image AprilTag pose or map-reporting products may still be published
one-for-one with incoming image or `apriltags` messages, including images with
no visible tags. Those outputs are reporting products only; the estimator
remains a single shared-state EKF.

### 7.3 Covariance policy

Published covariances come from:

- `Σ_y = J P Jᵀ`

All published covariance blocks remain full.

---

## 8. Parameters

Frames:

- `world_frame_id`
- `odom_frame_id`
- `base_frame_id`
- `imu_frame_id`
- `camera_frame_id`

Topics:

- `topics.imu`
- `topics.gravity`
- `topics.apriltags`
- `topics.camera_info`

Timing:

- fixed-lag buffer duration
- clock-jump reset threshold
- maximum allowed IMU gap for propagation

Noise:

- process noise for velocity, attitude, and bias walks
- IMU attitude acceptance/gating parameters
- gravity residual noise and gating parameters
- AprilTag reprojection noise and gating thresholds
