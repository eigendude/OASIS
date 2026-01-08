# EKF Design Document

This document defines the state-space model and time-handling policy for a localization EKF. The EKF is a **single optimizer over a shared state** with **multiple models**:

- a continuous-time **process model** (propagation), and
- discrete-time **measurement models** (updates), all applied on a common time axis.

Initial fusion: `imu_raw` + `imu_calibration` + `magnetic_field` + `apriltags`

Later: add visual constraints (ORB-SLAM3) without redesign.

The general architecture is single optimizer, lots of models. This keeps the state clean, and still gives good behavior in practice.

### World frame contract

The EKF maintains two frames:

- `world` (global, drift-corrected): long-term consistent frame (may be discontinuous).
- `odom` (local, continuous): smooth frame for control/navigation consumers.

TF tree contract: `world -> odom -> base_link`.

**AprilTag world correction policy**

AprilTag pose updates correct drift in the global `world` frame while keeping the
local `odom` frame continuous. The EKF state represents `T_world_base`, while a
separate correction transform `T_world_odom` keeps `odom -> base_link` smooth.

On an accepted AprilTag update:

- Let `T_odom_base` be the propagated (continuous) pose before the update.
- Apply the EKF update to produce `T_world_base_updated`.
- Update the correction so the odom pose is preserved:

```
T_world_odom = T_world_base_updated * inverse(T_odom_base)
```

This guarantees `T_world_odom * T_odom_base == T_world_base_updated` without
introducing discontinuities in the odom frame.

---

## 1. ROS Inputs

### 1.1 Streams (initial version)

- Single `message_filters` combination (ExactTime) producing an **IMU packet**:
  - `imu_raw` (`sensor_msgs/Imu`): inertial sample stream (time-stamped) — **drives propagation**
  - `imu_calibration` (`oasis_msgs/ImuCalibration`): calibration priors + uncertainty
    - Packaged alongside `imu_raw` for wiring simplicity.
    - **Semantic rule:** used **only once** to initialize the in-state calibration parameters (initial prior).
    - After initialization, `imu_calibration` contents are ignored (except diagnostics / consistency checks).
- `magnetic_field` (`sensor_msgs/MagneticField`): magnetometer samples (time-stamped) — measurement updates
- `apriltags` (`apriltag_msgs/AprilTagDetectionArray`) (time-stamped from raw camera image) — measurement updates
  - May contain detections of **multiple tags** across **multiple families**.
  - **Tag identity key:** `TagKey := (family, id)` (not `id` alone).
  - Family must be provided by the detector.
- `camera_info` (`sensor_msgs/CameraInfo`): camera intrinsics for the AprilTag pixel reprojection model
  - Subscribed once and cached (first valid message); treated as fixed for the session.
  - The EKF does not require `camera_info` to be time-synchronized with detections (intrinsics are static).

---

### 1.2 `imu_raw` — `sensor_msgs/Imu`

Used fields:

- `header.stamp` (`t_meas`)
- `header.frame_id` → IMU frame `{I}`
- `angular_velocity` `ω_raw` and `angular_velocity_covariance` `Σ_ω_raw`
- `linear_acceleration` `a_raw` and `linear_acceleration_covariance` `Σ_a_raw`

Ignored:

- `orientation`, `orientation_covariance`

Requirements:

- Covariances are interpreted as **per-sample noise**.
- `ω_raw`, `a_raw` are expressed in `{I}`.

---

### 1.3 `imu_calibration` — `oasis_msgs/ImuCalibration`

Role: **one-shot initialization prior** for systematic parameters that live in the shared state.

Validity:

- If `valid == false`, ignore.

Frame:

- `header.frame_id` must match `imu_raw.header.frame_id` (calibration is defined in `{I}`).

Models (parameter meaning):

- Accel: `a_corr = A_a (a_raw − b_a)` where `b_a, A_a` are _in-state parameters_
- Gyro: `ω_corr = ω_raw − b_g` where `b_g` is an _in-state parameter_

Requirements:

- `imu_calibration` is consumed **only while the filter is uninitialized**:
  - The **first** valid calibration observed is applied as an initial prior on the in-state calibration parameters
    (`b_a, A_a, b_g`) with its covariance.
  - After the EKF is initialized, **all subsequent `imu_calibration` messages are ignored** (except for diagnostics).
- Calibration uncertainty is **systematic** (parameter uncertainty), not sample noise.
- Always use full covariance matrices (don't diagonalize) when possible.

Runtime usage:

- For propagation, use the current estimated calibration parameters from the EKF state (which evolve via the filter),
  not the contents of `imu_calibration` messages.

---

### 1.4 `magnetic_field` — `sensor_msgs/MagneticField`

Used fields:

- `header.stamp` (`t_meas`)
- `header.frame_id` → mag frame `{M}`
- `magnetic_field` `m_raw`
- `magnetic_field_covariance` `Σ_m_raw`

Requirements (measurement convention + noise interpretation):

- **Convention:** form the magnetometer residual in the sensor frame `{M}` (do **not** rotate `m_raw` into `{B}`).
  - Measurement: `z := m_raw` in `{M}`
  - Predicted measurement: `z_hat := m̂_M` in `{M}`
  - Innovation: `ν := z - z_hat` in `{M}`
- `m_raw` and `Σ_m_raw` are interpreted in `{M}`.
  - The adaptive covariance `R_m(t)` is maintained in `{M}` and is directly comparable to `Σ_m_raw`.
- Use the estimated extrinsic `T_BM` (mag → body) to predict `m̂_M`:
  - Let `R_BM` be the rotation of `T_BM`, and let `R_BW` be world → body (from the current attitude estimate).
  - With an expected Earth field vector `m_W` in `{W}`, predict:
    `m̂_M = R_BMᵀ * R_BW * m_W`
  - `m_W` may be configured as a fixed vector (magnitude + direction) or modeled/estimated as a slowly-varying parameter,
    but the residual is always computed in `{M}`.
- `Σ_m_raw` is interpreted as the driver’s estimate of the magnetometer measurement noise covariance (the `R` term),
  but it is **not** treated as authoritative per-sample truth.

Initialization + runtime noise policy:

- On startup, initialize the magnetometer noise prior `R_m0` from the **first valid** `Σ_m_raw` observed
  (otherwise fall back to a configured default `R_m0`).
- After initialization, the EKF uses an internal adaptive covariance model `R_m(t)` updated from innovations.
- Constrain the adaptive estimate to remain symmetric positive definite and bounded:
  `R_min ⪯ R_m(t) ⪯ R_max`.

Default magnetometer noise prior (if no valid `Σ_m_raw` observed):

- Assume `σ_m = 20 µT` per axis (RMS).
- Set `R_m0 = diag([4e-10, 4e-10, 4e-10])` (tesla²).

Earth’s field magnitude is ~25–65 µT depending on location; the noise after filtering and in a clean setup is often single-digit µT RMS, but indoor interference pushes it higher.

Bounds for adaptive magnetometer covariance:

- `R_min = diag([ (1 µT)², (1 µT)², (1 µT)² ]) = diag([1e-12, 1e-12, 1e-12])` (tesla²)
- `R_max = diag([ (50 µT)², (50 µT)², (50 µT)² ]) = diag([2.5e-9, 2.5e-9, 2.5e-9])` (tesla²)

Notes:

- `R_m0` / bounds are expressed in tesla². Use µT intuition: `1 µT = 1e-6 T`.

Adaptive update (covariance matching):

Let the innovation at time k be `ν_k` with predicted innovation covariance `Ŝ_k = HPHᵀ` (excluding `R`).
Update:

`R_m ← clamp_SPD( (1-α) R_m + α (ν_k ν_kᵀ - Ŝ_k), R_min, R_max )`

with `α ∈ (0,1)` (default `α = 0.01`), and `clamp_SPD` enforces symmetry + eigenvalue clamping.

---

### 1.5 `apriltags` — `apriltag_msgs/AprilTagDetectionArray` (multi-tag landmark SLAM mode)

Used fields:

- `header.stamp` (`t_meas`)
- `header.frame_id` → camera frame `{C}`
- For each detection:
  - `family` (string or enum; must be provided by the detector)
  - `id` (integer tag ID)
  - `corners[4]` (pixel coordinates)
  - `decision_margin` (gating)
  - `homography[9]` (3×3 row-major) (pre-gating / initialization aid)

Definitions:

- `TagKey := (family, id)`.

Semantic role:

- AprilTags provide **pixel-space constraints** that jointly update:
  - global body pose `T_WB` (and thus `T_WO`), and
  - camera extrinsics `T_BC`, and
  - **tag landmark poses** `T_WTag(TagKey)` for any observed tags.

All tags are **fully learned**:

- `T_WTag(TagKey)` is **in-state** for each tag that has been initialized.
- Each tag landmark has a **full 6×6 covariance** in the EKF state (in tangent space).

World anchor contract (TagKey = (anchor_family, anchor_id)):

- The anchor tag is `TagKey0 := (tag_anchor_family, tag_anchor_id)`.
- The EKF defines `world` such that `T_WTag(TagKey0)` is initialized with:
  - Mean: identity transform
  - Translation prior stddev: `σ_t = tag_landmark_prior_sigma_t_m` per axis
  - Rotation prior stddev: `σ_rot = tag_landmark_prior_sigma_rot_rad` per axis (tangent space)
- This is an initial prior (broad but finite).

Camera model requirement:

The AprilTag measurement model operates in pixel space (corner reprojection), so the EKF MUST know the
camera intrinsics `K` and distortion model `D`.

Source of `K, D`:

- Subscribe to `sensor_msgs/CameraInfo` and cache the first valid message.
- Require `camera_info.header.frame_id == apriltags.header.frame_id` so the cached intrinsics are associated with
  the same camera frame `{C}` used by detections.
  intrinsics are associated with the same camera frame `{C}` used by detections.
- Treat cached `K, D` as fixed for the session (no online re-calibration).
- If `camera_info` has not been cached yet, AprilTag updates are rejected (with diagnostics) until it is available.
- If a later `camera_info` arrives with different `K`/`D`, ignore it and emit a warning.
- Use `camera_info.distortion_model` + `D` as provided. If distortion is non-empty but unsupported by the EKF’s
  projection code, reject AprilTag updates and emit diagnostics until a supported model is available.

Tag geometry:

- The EKF must know the physical size of the tag.
- **Always use one global tag size:** `tag_size_m` (meters), configured once and applied to all detections.

Measurement model (per detection i):

- Measurement vector: stacked pixel corners
  - `z_i := [u0 v0 u1 v1 u2 v2 u3 v3]ᵀ`
- Predicted corners are obtained by projecting the known 3D tag corner points (defined by `tag_size_m`) from
  `{TagKey_i}` into camera pixels:
  - Use current estimates of `T_WB`, `T_BC`, and `T_WTag(TagKey_i)`:
    - `T_WC = T_WB * inverse(T_BC)` (camera pose in world)
    - `T_CTag = inverse(T_WC) * T_WTag(TagKey_i)`
  - Project each corner point through distortion model `D` and intrinsics `K` into pixel coordinates.
- Innovation: `ν_i := z_i - ẑ_i` in pixels.

Multi-tag update (SEQUENTIAL):

- For one incoming `AprilTagDetectionArray` message, form candidate updates for **all detections** that pass gating.
- **Implementation MUST apply per-tag sequential EKF updates** in deterministic order:
  - Deterministic ordering: sort by `(family, id)`.
  - **Fixed linearization point per message:** for this one `AprilTagDetectionArray`, compute each accepted tag’s
    residual/Jacobian using the same predicted state `x̂(t_meas)⁻` (post-propagation to `t_meas`, pre-AprilTag updates).
  - Apply updates sequentially (one tag at a time) to produce the final `(x̂(t_meas)⁺, P(t_meas)⁺)`.

Measurement noise (initial prior):

- Pixel-domain corner noise prior (per detection):
  - `R_tag = (σ_px^2) I_8`, default `σ_px = 1.0 px`
- `R_tag` may be inflated per detection using quality heuristics:
  - lower `decision_margin` → larger `σ_px`
  - near-singular homography → reject or inflate

Gating:

- Gate each detection on reprojection error:
  - reject if RMS reprojection error > `e_px_max`
- Gate on Mahalanobis distance:
  - `d² = νᵀ S^{-1} ν` vs `χ²` threshold (8 dof per tag if using 8-dim corners).

Homography usage:

The provided `homography` must be used for:

- fast pre-gating / sanity checks (e.g., reject near-singular projective warps),
- deriving an initial pose proposal for a planar tag solve (used only for initialization / linearization seed),
- heuristically inflating `R_tag` when detection quality is low.

Note: `homography` is never fused directly as a measurement.

Landmark initialization policy (new TagKey):

When a `TagKey = (family, id)` is observed for the first time (no state for `T_WTag(TagKey)` exists yet):

1. Use homography + known tag size (`tag_size_m`) + `K` to obtain an initial `T_CTag(TagKey)` proposal.
2. Compose into world using the current camera pose estimate:
   - `T_WTag(TagKey)_init = T_WC * T_CTag(TagKey)_proposal`
3. Insert `T_WTag(TagKey)` into the EKF state with a broad but finite prior covariance:
   - Mean: `T_WTag(TagKey)_init`
   - Translation prior stddev: `σ_t = tag_landmark_prior_sigma_t_m` per axis
   - Rotation prior stddev: `σ_rot = tag_landmark_prior_sigma_rot_rad` per axis (tangent space)
4. Immediately apply the pixel-corner measurement update for that detection (same timestamp),
   as part of the same sequential per-tag update pass.

This makes all tags “fully learned” with full covariances, while using `TagKey0` as the origin anchor.

---

## 2. Frames and Extrinsics

Frames:

- `{W}`: global world frame, ROS frame_id = `world`
- `{O}`: local continuous frame, ROS frame_id = `odom`
- `{B}`: EKF body frame, ROS child_frame_id = `base_link`
- `{I}`: IMU measurement frame (`imu_raw.header.frame_id`)
- `{M}`: magnetometer frame (`magnetic_field.header.frame_id`)
- `{C}`: camera frame (`apriltags.header.frame_id`)
- `{Tag_(family,id)}`: tag-local frame for a tag (internal; not necessarily published as TF)

Estimated extrinsics (in-state):

- `T_BI` (IMU → body)
- `T_BM` (mag → body)
- `T_BC` (camera → body)

Estimated landmarks (in-state):

- For each initialized `TagKey`: `T_WTag(TagKey)` (tag pose in world), with full covariance.

Parameterization:

- Rotation: quaternion
- Translation: 3-vector
- Uncertainty: represented in tangent space (6D pose error: δρ, δθ)

Initialization requirement:

- Because `{B}` is defined independently of `{I}`, `T_BI` MUST be initialized with a **broad but finite prior**
  (mean + covariance). Without this prior, the relative alignment between `{B}` and `{I}` is undefined at startup.

Default `T_BI` prior:

- Mean: identity transform.
- Rotation prior stddev: `σ_rot = π rad` per axis (tangent space).
- Translation prior stddev: `σ_t = 1.0 m` per axis.

Default extrinsic priors (`T_BM`, `T_BC`):

- Mean: identity transform.
- Rotation prior stddev: `σ_rot = π rad` per axis.
- Translation prior stddev: `σ_t = 1.0 m` per axis.

Default landmark priors (`T_WTag(TagKey)`):

- For anchor tag `TagKey0`: mean identity, `σ_t = tag_landmark_prior_sigma_t_m`, `σ_rot = tag_landmark_prior_sigma_rot_rad`.
- For any newly initialized tag: mean from initialization solve, same `σ_t`, `σ_rot` as configured.

---

## 3. Time Axis, Buffer, and Deterministic Replay (Fixed-lag)

### 3.1 Time definitions

- `t_meas`: `header.stamp` from the incoming message (authoritative event time)
- `t_now`: receipt time (diagnostics only)
- `t_filter`: max `t_meas` among **accepted** messages (the frontier)

The EKF is event-driven: each accepted message attaches at `t_meas`. Out-of-order messages are supported via fixed-lag replay.

---

### 3.2 Message roles

- **Propagation input:** `imu_raw` (arrives inside the IMU packet).
- **Initialization prior (one-shot):** `imu_calibration` (arrives inside the IMU packet; used only before initialization).
- **Measurement updates:** `magnetic_field`, `apriltags` (and later other vision sources).

Notes:

- The IMU packet is a transport/coordination convenience.
- Only `imu_raw` drives propagation at runtime.
- `imu_calibration` is an initialization prior only; it never “feeds” per-sample calibration after init.

---

### 3.3 IMU packet synchronization contract

The EKF consumes IMU as a synchronized pair:

- `imu_pkt = (imu_raw, imu_calibration)` produced by `message_filters` using **ExactTime**.

Timestamp policy:

- Packet time: `t_meas := imu_raw.header.stamp` (authoritative event time for propagation).
- Require `imu_calibration.header.stamp == imu_raw.header.stamp`; reject the packet if not.

Semantic policy:

- `imu_raw` is used every packet for propagation (after applying the current in-state calibration parameters).
- `imu_calibration` is used **only to initialize** the calibration parameters if the EKF is not yet initialized.
  After initialization, `imu_calibration` is ignored (except diagnostics / consistency checks).

---

### 3.4 Buffer model

Maintain a time-ordered ring buffer of time nodes over a fixed lag `T_buffer_sec`.

Each node at `t_k` stores:

- state `x_k`, covariance `P_k`
- all measurement messages attached at `t_k` (mag, tags, etc.)
- IMU samples needed to propagate forward (IMU packets spanning intervals)

Node rules:

- One node per distinct timestamp (multiple messages can attach to the same node).
- Evict nodes older than `t_filter - T_buffer_sec` (with drop counters / diagnostics).

---

### 3.5 Propagation convention (explicit)

Propagation uses IMU packets with **zero-order hold (ZOH)**:

- Treat the corrected IMU inputs computed from `imu_raw` using the **current in-state calibration parameters**
  as piecewise-constant from one IMU timestamp to the next.
- For each IMU interval `[t_i, t_{i+1}]`, propagate using constant inputs evaluated at `t_i`.

Calibration timing (in-state, not message-driven):

- The calibration parameters (`b_g`, `b_a`, `A_a`, and any other calibration terms in-state) are taken from the EKF state.
- During propagation over `[t_i, t_{i+1}]`, hold these parameters constant (ZOH) at their values at `t_i`.
- `imu_calibration` messages never provide the per-interval calibration.
  The **first valid** `imu_calibration` is consumed only to initialize the in-state calibration parameters.
  After initialization, `imu_calibration` is ignored and does not produce any updates during replay.

Strict coverage rule:

- The EKF only propagates across an interval if IMU samples cover it without a gap exceeding `Δt_imu_max`.
- If `Δt > Δt_imu_max` occurs inside the required interval, that interval is **not propagatable** and fixed-lag replay across it is rejected.

---

### 3.6 Insertion + replay (deterministic)

On message arrival:

1. **Validate timestamp**
   - Reject if missing/invalid.
   - Reject if `t_meas > t_now + ε_wall_future`.

2. **Fixed-lag window**
   - If `t_filter` exists and `t_meas < t_filter - T_buffer_sec`, reject as too old.

3. **Attach**
   - Insert/attach the message at node `t_meas` (create node if needed).

4. **Update frontier / replay**
   - If `t_meas > t_filter`: advance `t_filter` and propagate forward using IMU ZOH.
   - Else: apply update at `t_meas`, then replay forward to `t_filter` using stored IMU samples.

Per-node application order (stable):

1. Apply any priors/parameter constraints attached at `t_k`
2. Apply measurement updates at `t_k` (mag, tags, etc.) in stable tie-break order:
   - lexicographic `(message_type, topic, frame_id)`
3. Within an AprilTag update at `t_k`, apply **sequential per-tag updates** ordered by `(family, id)` with a fixed
   linearization point (Section 1.5).

Replay must never depend on arrival order.

---

### 3.7 Drop / reset rules

Reject and emit diagnostics if:

- missing timestamp
- required covariance contains NaN/Inf
- message outside fixed-lag window
- propagation coverage insufficient (`Δt_imu_max` violated)
- clock discontinuity detected: reset filter + buffer if `|t_meas - t_filter| > Δt_clock_jump_max`
- AprilTag update rejected if `apriltags.header.frame_id` does not match the cached `camera_info.header.frame_id`
  (camera frame mismatch).
- If a new `camera_info` arrives with different `K`/`D`, emit a warning and keep the cached one.
- AprilTag update rejected if `apriltags.header.frame_id` does not match `{C}` (per `camera_frame_id` policy),
  or if cached `camera_info.header.frame_id` does not match `{C}`.

---

## 4. Outputs (poses + covariances + measurement stats)

The EKF publishes:

1. state estimates in ROS-standard frames (`world`, `odom`, `base_link`)
2. covariances for those estimates in each published frame
3. per-measurement innovation / gating statistics (so “all measurements” have covariances too)

### 4.1 Frame tree

TF tree contract: `world -> odom -> base_link`

- `odom` is continuous (no jumps).
- `world` is global and may jump when corrected by global measurements (e.g., AprilTags).

### 4.2 State estimate outputs (with covariances in BOTH frames)

**(A) `nav_msgs/Odometry` in `odom` (continuous local output)**

- Topic: `odom`
- `header.stamp = t_filter`
- `header.frame_id = "odom"`
- `child_frame_id = "base_link"`
- Pose: `T_OB`
- Twist: expressed in `{B}` (default)
- Covariance: derived from EKF covariance `P` and expressed in the message’s frame convention.

**(B) `nav_msgs/Odometry` in `world` (global output with covariances)**

- Topic: `world_odom` (or `global_odom`)
- `header.stamp = t_filter`
- `header.frame_id = "world"`
- `child_frame_id = "base_link"`
- Pose: `T_WB`
- Twist: same convention as `odom` message (declare it and keep consistent)
- Covariance: derived from the same `P` (see 4.4).

**(C) TF**

- Broadcast `TF: odom -> base_link` from `T_OB` (`stamp = t_filter`)
- Broadcast `TF: world -> odom` from `T_WO` (`stamp = t_filter`)

Definition:
`T_WO := T_WB * inverse(T_OB)` so `T_WB = T_WO * T_OB`

Note: TF carries no covariance; covariances are provided by (A) and (B).

### 4.3 Extrinsic and landmark outputs (with covariances)

Extrinsics (optional, if consumers need uncertainty):

- `geometry_msgs/PoseWithCovarianceStamped`:
  - Topic: `T_BI`, `T_BM`, `T_BC` (namespaced)
  - `header.stamp = t_filter`
  - `header.frame_id = "base_link"`
  - Pose: rotation+translation of the sensor in `{B}`
  - Covariance: derived from `P`

Landmarks (optional, for debugging / mapping):

- `geometry_msgs/PoseWithCovarianceStamped` per tag (or an array/custom message):
  - Topic: `tags/<family>/<id>/pose` (namespaced)
  - `header.stamp = t_filter`
  - `header.frame_id = "world"`
  - Pose: `T_WTag(TagKey)`
  - Covariance: derived from `P` (full 6×6 in tangent space mapped to message convention as policy dictates)

Do NOT rely on TF for extrinsic/landmark covariances; TF cannot represent them.

### 4.4 Covariance transformation rules (how “all frames” is satisfied)

The EKF maintains a single covariance `P` over its internal error-state. Any published covariance is obtained by
linearizing the published quantity `y = g(x)` at the current mean:

`Σ_y = J * P * Jᵀ`, where `J = ∂g/∂x` evaluated at `x̂`.

For common frame changes (rotation only), use:

- Position covariance transform:
  `Σ_pA = R_AB * Σ_pB * R_ABᵀ`
- Small-angle orientation covariance transform:
  `Σ_θA = R_AB * Σ_θB * R_ABᵀ`
- Twist covariance transform:
  `Σ_vA = R_AB * Σ_vB * R_ABᵀ`, `Σ_ωA = R_AB * Σ_ωB * R_ABᵀ`

For full SE(3) pose errors in a 6×6 tangent (δρ, δθ), use the adjoint:

`Σ_A = Ad_{T_AB} * Σ_B * Ad_{T_AB}ᵀ`

Implementation note:

- The Odometry message covariance is a 6×6 for pose and 6×6 for twist; fill only the parts actually estimated,
  and set others large or NaN-free “unknown” values per policy.

### 4.5 Per-measurement outputs (so “ALL measurements” have covariances)

For every accepted or rejected measurement update (mag, tags, later vision), publish a measurement report:

Recommended topic:

- `ekf/updates/<sensor>` (e.g., `ekf/updates/mag`, `ekf/updates/apriltags`)

Recommended contents:

- `header.stamp = t_meas`
- measurement id (tag id / sensor instance)
- `z` (measurement vector)
- `z_hat` (predicted measurement)
- innovation `ν = z - z_hat`
- measurement noise used `R` (the covariance actually used for this update)
- predicted innovation covariance excluding R: `Ŝ = HPHᵀ`
- total innovation covariance: `S = Ŝ + R`
- Mahalanobis distance: `d² = νᵀ S^{-1} ν`
- gating threshold used and accept/reject flag

For AprilTags:

- Publish **per-detection** entries (one per `TagKey`) from a single `AprilTagDetectionArray`.
- Entries MUST reflect the actual sequential processing order `(family, id)` and the `R_tag` inflation applied per tag.

---

## 5. Config Parameters (additions)

In addition to the priors you already specified:

- Output frames:
  - `world_frame_id` (default `"world"`)
  - `odom_frame_id` (default: `"odom"`)
  - `body_frame_id` (default `"base_link"`)

AprilTag multi-tag / landmark parameters:

- `tag_size_m` (global tag size, in meters, required)
- `tag_anchor_id` (default: `0`)
- `tag_anchor_family` (default: `tag36h11`)
- `tag_landmark_prior_sigma_t_m` (default: `10.0`)
- `tag_landmark_prior_sigma_rot_rad` (default: `pi`)

Additionally, define these as parameters with the respective defaults we should use:

- `T_buffer_sec`
- `ε_wall_future`
- `Δt_clock_jump_max` (needed for reset-on-jump)
- `Δt_imu_max`
