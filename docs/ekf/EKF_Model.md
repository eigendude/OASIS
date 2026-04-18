# EKF Model Specification

This document defines the target structure for the localization EKF rewrite
around fused `imu` + `gravity` + `apriltags` + `camera_info`.

The EKF is a shared-state estimator. Core logic should stay ROS-agnostic and
operate on normalized sensor packets, deterministic update ordering, and full
covariance transport throughout the stack.

This document is about implementation structure. Runtime localization behavior
and ownership belong in `EKF.md`. The focused AprilTag measurement and map
contract belongs in `EKF_AprilTags.md`.

---

## 1. Directory layout target

Create the EKF implementation under `oasis_control/oasis_control` with clear
separation between shared reusable localization utilities, EKF-specific
estimation code, and ROS wiring.

Suggested structure:

```text
oasis_control/oasis_control/localization/common/
  algebra/
    quat.py
    linalg.py
    covariance.py

  frames/
    mounting.py
    frame_policy.py
    camera_extrinsics.py

  data/
    imu_sample.py
    gravity_sample.py

  validation/
    imu_validation.py
    gravity_validation.py

  measurements/
    gravity_direction.py

oasis_control/oasis_control/localization/ekf/
  math/
    se3.py
    camera.py
    projection.py

  data/
    apriltag_packet.py
    camera_intrinsics.py
    ekf_state.py
    diagnostics.py
    update_report.py

  state/
    error_state.py
    state_mapping.py
    landmark_index.py

  process/
    process_model.py
    imu_attitude_update.py
    gravity_update.py

  vision/
    apriltag_update.py
    landmark_initializer.py

  filter/
    ekf.py
    predict_step.py
    update_step.py

  replay/
    replay_engine.py
    ring_buffer.py
    timeline_node.py

  config/
    ekf_config.py
    ekf_params.py

oasis_control/oasis_control/nodes/ekf/
  ekf_node.py
  ros_messages.py
  ros_publishers.py
  ros_params.py
  tf_bridge.py

oasis_control/test/ekf/
  common/
  state/
  process/
  vision/
  filter/
  replay/
```

Rules:

- core modules contain no ROS wiring, TF lookups, QoS policy, or topic code
- node code handles ROS messages, parameters, TF, QoS, diagnostics, and
  publication
- package `__init__.py` files stay empty

Why this split is better:

- `localization/common/` gives EKF and AHRS one explicit shared layer for IMU,
  gravity, frame-policy, covariance, and gravity-direction logic
- `common/data/` owns the canonical IMU and gravity packet types used by both
  components
- `common/frames/` owns fixed mounting and frame-policy helpers that enforce
  the shared `imu_link` and `base_link` conventions
- `common/validation/` owns reusable IMU and gravity validation tied to the
  shared sensor contract
- `common/measurements/` owns reusable gravity-direction residual math used by
  both AHRS consistency checks and EKF gravity updates
- `localization/ekf/` stays focused on state layout, process/update flow,
  replay, and vision-specific logic instead of duplicating low-level utilities

What should not move into `common/`:

- EKF replay machinery
- landmark map and covariance ownership
- AprilTag reprojection and landmark initialization
- filter scheduling and shared-state ownership
- EKF-specific diagnostics and reporting when they diverge from AHRS needs

---

## 2. Shared conventions

Frames:

- `world`
- `odom`
- `base_link`
- `imu_link`
- `camera_link`

TF contract:

`world -> odom -> base_link`

Policy:

- `odom -> base_link` remains continuous
- `world -> odom` may change when AprilTag updates correct global drift
- `imu` samples are expected from `imu_link`
- `gravity` samples are expected from `imu_link`
- `T_BI` is the fixed transform from `imu_link` to `base_link`
- `apriltags` are expected from `camera_link`
- `camera_info` is expected from `camera_link`
- `T_BC` is the fixed transform from `camera_link` to `base_link`
- the current default source for `T_BI` is the boot-time mounting calibration
  contract in `AHRS_Mounting.md`
- frame-policy enforcement belongs in `localization/common/frames/`

Transform naming convention:

- `T_XY` maps quantities from frame `{Y}` into frame `{X}`
- equivalently, `p_X = T_XY p_Y` in homogeneous-transform notation
- under this convention, `T_BI` maps `imu_link -> base_link` and `T_BC` maps
  `camera_link -> base_link`

Covariances:

- preserve full covariance throughout
- never diagonalize by convenience
- rotate or transport covariance explicitly when measurements move between
  frames

Determinism:

- updates from one AprilTag message are applied in sorted `(family, id)` order
- fixed-lag replay must produce deterministic state and covariance when the
  same input sequence is replayed

---

## 3. Core data types

The package boundary should read as:

- `localization/common/data/` for the canonical IMU and gravity packets shared
  with AHRS
- `localization/common/frames/` for fixed mounting, camera extrinsics, and
  frame-policy helpers
- `localization/common/validation/` for reusable IMU and gravity validation
- `localization/common/measurements/` for reusable gravity-direction
  measurement math
- `localization/ekf/data/` for EKF-specific packets, state snapshots, and
  update-facing reports
- `localization/ekf/state/` for covariance layout, error-state indexing, and
  landmark state bookkeeping
- `localization/ekf/process/` for EKF motion and measurement-update models
- `localization/ekf/vision/` for AprilTag reprojection and landmark
  initialization
- `localization/ekf/replay/` for fixed-lag buffering and deterministic
  re-execution

### 3.1 `ImuSample`

Fields:

- timestamp in integer nanoseconds
- IMU frame id, expected to be `imu_link`
- normalized quaternion `q_WI`
- optional orientation covariance
- angular velocity vector and covariance
- linear acceleration vector and covariance

### 3.2 `GravitySample`

Fields:

- timestamp in integer nanoseconds
- source frame id, expected to be `imu_link`
- physical gravity vector in `imu_link` that points down and is near
  `9.81 m/s^2` at rest
- full `3 x 3` covariance

This type is a first-class gravity measurement packet, not an "up" vector, not
a normalized unit vector, and not a propagation input.

### 3.3 `AprilTagPacket`

Fields:

- timestamp
- frame id, expected to resolve to `camera_link`
- sorted list of detections
- each detection keyed by `TagKey := (family, id)`
- image corners
- optional decision or quality metrics used for gating

The detailed AprilTag measurement contract stays in `EKF_AprilTags.md`.

### 3.4 `EkfState`

Fields:

- `p_WB`
- `v_WB`
- `q_WB`
- `b_g`
- `b_a`
- `T_BI`
- `T_BC`
- `T_WTag(TagKey)` for initialized landmarks
- optional `g_W`

The associated covariance is full and matches the active state layout. Fixed
extrinsics such as `T_BC` may be stored with the state snapshot without
becoming online-estimated covariance blocks.

### 3.5 `CameraIntrinsics`

Fields:

- frame id, expected to be `camera_link`
- image width and height
- pinhole intrinsics matrix terms from `camera_info`
- distortion model and coefficients when needed by the chosen projection path

This type owns the normalized camera model cached from `camera_info`.

### 3.6 `UpdateReport`

Fields:

- update timestamp
- update kind
- accepted or rejected status
- innovation summary
- gating summary
- affected state blocks

This type is the bridge between estimator internals and ROS diagnostics or
per-update reporting.

---

## 4. Pipeline model

### 4.1 Predict on `imu`

The predict path advances the nominal state and covariance on each accepted IMU
sample.

Responsibilities:

- validate and normalize the fused quaternion
- map angular velocity and linear acceleration from `imu_link` into `base_link`
- propagate `p_WB`, `v_WB`, `q_WB`, and bias terms
- discretize and carry full process covariance

This path belongs under `process/` because it is part of the motion/update
model family, not the filter scheduling shell.

Shared sample validation, frame-policy enforcement, mounting application, and
covariance transport helpers should come from `localization/common/`.

### 4.2 IMU attitude update

The fused quaternion is treated as a direct attitude observation of the IMU
frame and mapped through `T_BI`.

Responsibilities:

- compute the predicted IMU attitude from the current base attitude and
  extrinsics
- form a minimal tangent-space residual
- preserve full covariance from the incoming orientation block when valid

This remains EKF-specific because it updates the shared localization state,
even though it depends on common quaternion, mounting, and validation helpers.

### 4.3 Gravity update

The gravity update is first-class and required.

Responsibilities:

- map the measured gravity vector from `imu_link` into `base_link`
- predict base-frame gravity direction from the current state using the shared
  convention `WORLD_GRAVITY_DIRECTION = (0, 0, -1)`
- compute a gravity-direction residual
- use full covariance from the message after frame transport
- support roll/pitch stabilization and IMU-to-base mounting observability

This model must not claim absolute yaw information from gravity.

The reusable gravity-direction residual math should live in
`localization/common/measurements/gravity_direction.py`. EKF-specific update
assembly, Jacobians, and covariance injection remain in
`localization/ekf/process/gravity_update.py`.

### 4.4 AprilTag reprojection update

The AprilTag path remains image-space and landmark-based.

Responsibilities:

- use `camera_info` intrinsics directly
- represent camera intrinsics in `data/camera_intrinsics.py`
- use fixed `T_BC` handling from `common/frames/camera_extrinsics.py`
- predict tag corner pixels from `T_WB`, fixed `T_BC`, and `T_WTag(TagKey)`
- update in deterministic sorted tag order
- allow decision metrics to affect gating or noise inflation

New landmark initialization may use homography or PnP bootstrap. The full
measurement and map policy is specified in `EKF_AprilTags.md`.

The implementation split should therefore be:

- `vision/apriltag_update.py` for the image-space residual and EKF-facing update
  assembly
- `vision/landmark_initializer.py` for first-seen landmark bootstrap policy
- `math/camera.py` for camera-model conventions tied to EKF vision use
- `math/projection.py` for projection and reprojection helpers used by the
  AprilTag update

Camera projection helpers may stay under `localization/ekf/math/` because they
are tightly coupled to the AprilTag reprojection path rather than to AHRS.

AprilTag update ownership should be explicit:

- `apriltag_packet.py` owns normalized detections in `camera_link`
- `camera_intrinsics.py` owns the cached intrinsics contract from
  `camera_info`
- `common/frames/camera_extrinsics.py` owns the fixed `T_BC` contract used by
  the EKF
- `apriltag_update.py` consumes intrinsics, fixed `T_BC`, and landmark state
  to form reprojection residuals in `camera_link`

### 4.5 Fixed-lag replay

Delayed AprilTag packets may arrive after later IMU or gravity samples.

The replay layer therefore owns:

- a ring buffer of time-ordered nodes
- measurement insertion by authoritative message time
- deterministic replay from the insertion point forward
- stable update ordering across replay runs

Using `replay/` instead of `timing/` makes it clearer that this package is not
general clock utilities; it owns delayed-measurement buffering and rerun logic.

Replay stays out of `common/` because AHRS does not own fixed-lag history or
late-arriving landmark updates.

Suggested ROS-layer responsibilities:

- `ros_messages.py` maps ROS inputs into `localization/common/data/` and
  `localization/ekf/data/` packets and maps estimator outputs into ROS-facing
  messages
- `ros_publishers.py` owns odometry/state/update publishers plus diagnostics
- `ros_params.py` owns parameter declaration and validation for topics, frames,
  replay, and noise settings
- `tf_bridge.py` owns `world -> odom -> base_link` publication policy

---

## 5. Testing targets

Unit and integration tests should cover:

- SE(3) math
- covariance transport
- gravity residual model
- AprilTag reprojection residuals
- landmark initialization
- replay determinism
- ROS conversion fidelity

Useful breakdown:

- `common/`: quaternion, covariance, frame-policy, validation, and
  gravity-direction identities shared with AHRS
- `process/`: predict path plus IMU-attitude and gravity residual correctness
- `vision/`: AprilTag reprojection and landmark initialization correctness
- `filter/`: predict/update consistency and covariance symmetry
- `replay/`: delayed-measurement replay order and determinism

---

## 6. Parameters

- input topic names for `imu`, `gravity`, `apriltags`, and `camera_info`
- frame ids for `world`, `odom`, `base_link`, `imu_link`, and `camera_link`
- fixed-lag duration and replay policy
- process noise and bias walk parameters
- IMU attitude gating parameters
- gravity gating parameters
- AprilTag reprojection noise and acceptance thresholds
