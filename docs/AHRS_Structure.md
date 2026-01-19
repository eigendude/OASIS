# AHRS Structure Specification

This document defines the target structure, component boundaries, and
mathematical specification for a full rewrite of the AHRS implementation.
It is intended to be implemented in small, verifiable steps where each
component has explicit inputs/outputs and documented math.

The goal is to move from a flat, function-centric layout to a class-based,
modular system. The core algorithms must be ROS-agnostic. ROS-specific
integration must be thin and localized.

---

## 1. Directory layout (target)

Create a new AHRS package layout under `oasis_control/oasis_control`, with
subdirectories to separate core math, estimation, and ROS integration.
Suggested structure (names are flexible but should follow the intent):

```
oasis_control/oasis_control/localization/ahrs/
  math/
    quat.py
    se3.py
    linalg.py
    stats.py
    units.py

  models/
    imu_model.py
    mag_model.py
    process_model.py
    noise_adaptation.py
    extrinsics_model.py

  state/
    ahrs_state.py
    error_state.py
    covariance.py
    state_mapping.py

  filter/
    ekf.py
    update_step.py
    predict_step.py

  time/
    time_base.py
    timeline.py
    ring_buffer.py
    replay_engine.py

  config/
    ahrs_config.py
    ahrs_params.py

  io/
    imu_packet.py
    mag_packet.py
    diagnostics.py
    update_report.py

oasis_control/oasis_control/nodes/ahrs/
  ahrs_node.py
  ros_params.py
  ros_conversions.py
  ros_publishers.py
  ros_time.py

oasis_control/test/ahrs/
  math/
  models/
  state/
  filter/
  time/
```

All modules under `oasis_control/oasis_control/` should have `__init__.py` files.

**Key rules**:

- `oasis_control/oasis_control/localization/ahrs/` contains all math, state, and estimation logic.
- `oasis_control/oasis_control/nodes/ahrs/` contains all ROS types, parameter handling, logging, QoS, and topic
  wiring.
- `oasis_control/test/ahrs/` should avoid numpy and depend only on standard library + core code.

---

## 2. Shared math conventions

### 2.1 Frames and symbols

- `{W}`: world frame (`world`)
- `{O}`: odom frame (`odom`)
- `{B}`: body frame (`base_link`)
- `{I}`: IMU measurement frame
- `{M}`: magnetometer frame

Vectors are columns. Rotations use quaternions and rotation matrices.

### 2.2 Quaternion conventions

- Quaternion `q_WB` rotates vectors from `{W}` to `{B}` (world → body).
- Rotation matrix `R_WB` is the 3×3 matrix corresponding to `q_WB` such that:
  - `v_B = R_WB * v_W`
- The inverse rotation is:
  - `R_BW = R_WBᵀ`
- Quaternion action is defined by: `v_B = R(q_AB) * v_A`.
- Composition (apply `q_AC` then `q_CB`) is: `q_AB = q_CB ⊗ q_AC` (Hamilton product),
  consistent with: `R(q_CB ⊗ q_AC) = R(q_CB) * R(q_AC)`.

### 2.3 SE(3) perturbations

For any transform `T_AB = (R_AB, p_AB)`:

- Small perturbation is represented by a 6D vector:
  - `δξ = [δρ; δθ]` (translation, rotation)
- For rotation perturbation:
  - `R ≈ Exp(δθ) * R_hat`

Implement standard `Exp`/`Log` and `Adjoint` operators in
`oasis_control/oasis_control/localization/ahrs/math/se3.py`.

### 2.4 Covariances

- Covariances are full matrices, never diagonalized.
- Any covariance output to ROS must be generated as:
  - `Σ_y = J * P * Jᵀ`
- For SE(3) poses in tangent coordinates:
  - `Σ_A = Ad_{T_AB} * Σ_B * Ad_{T_AB}ᵀ`

---

## 3. Core state definition

Implement a **single shared state** representing navigation, calibration,
extrinsics, and environment reference vectors.

### 3.1 Mean state (`AhrsState`)

- `p_WB ∈ ℝ³` Position of body in world
- `v_WB ∈ ℝ³` Velocity of body in world
- `q_WB` Unit quaternion (world → body)
- `ω_WB ∈ ℝ³` Body angular rate (world → body), expressed in `{B}`
- `b_g ∈ ℝ³` Gyro bias (in `{I}`)
- `b_a ∈ ℝ³` Accel bias (in `{I}`)
- `A_a ∈ ℝ^{3×3}` Accel scale/misalignment matrix (acts in `{I}`)
- `T_BI ∈ SE(3)` IMU → body
- `T_BM ∈ SE(3)` Mag → body
- `g_W ∈ ℝ³` Gravity vector in world
- `m_W ∈ ℝ³` Magnetic field vector in world

### 3.2 Error state (`AhrsErrorState`)

The covariance `P` is defined over this stacked vector:

- `δp, δv, δθ ∈ ℝ³`
- `δω ∈ ℝ³` angular-rate error for `ω_WB` (in `{B}`)
- `δb_g, δb_a ∈ ℝ³`
- `δA_a ∈ ℝ⁹` (row-major perturbation)
- `δξ_BI, δξ_BM ∈ ℝ⁶` (SE(3) tangent)
- `δg_W, δm_W ∈ ℝ³`

`state_mapping.py` must provide deterministic ordering and mapping between
mean state, error state, and covariance blocks.

---

## 4. Process model (continuous-time)

The process model encodes **smooth motion** and **slow parameter drift**.
It does not integrate raw IMU measurements as inputs. IMU gyro, accel, and
magnetometer are applied as **measurement updates**.

### 4.1 Continuous dynamics

Navigation kinematics:

- `ṗ_WB = v_WB`
- `v̇_WB = w_v`

Interpretation:

- `w_v` is a zero-mean smoothness prior on world-frame linear acceleration (units: m/s²).
  In the mean propagation, `v̇_WB` is typically integrated with `E[w_v] = 0`.

Attitude kinematics (driven by latent angular rate):

- `q̇_WB = 0.5 * Ω(ω_WB) * q_WB`
- `ω̇_WB = w_ω`
- `w_ω` is a zero-mean smoothness prior on angular-rate change (units: rad/s²).

Definition of `Ω(ω)`:

`ω_WB` is expressed in `{B}`. We use quaternion kinematics consistent with the
action `v_B = R(q_WB) v_W`, with `q` stored as `[w, x, y, z]ᵀ`:

Let `ω = [ωx, ωy, ωz]ᵀ` and define the 4×4 matrix:

Ω(ω) =
[ 0   -ωx  -ωy  -ωz
  ωx   0    ωz  -ωy
  ωy  -ωz   0    ωx
  ωz   ωy  -ωx   0  ]

Then `q̇ = 0.5 * Ω(ω) * q`.

Systematic parameter drift (random walks):

- `ḃ_g = w_bg`
- `ḃ_a = w_ba`
- `Ȧ_a = W_A` where `vec(Ȧ_a) = w_A`
- `δξ̇_BI = w_BI`
- `δξ̇_BM = w_BM`
- `ġ_W = w_g`
- `ṁ_W = w_m`

Notes:

- This process is intentionally weak: it encodes “motion is smooth” and “rate is
  smooth”. The gyro measurement update provides the primary constraint on `ω_WB`
  (and indirectly on `q_WB` through propagation).

### 4.2 Noise intensities

Let `Q_c` be the continuous-time noise covariance for:

`[w_v, w_ω, w_bg, w_ba, w_A, w_BI, w_BM, w_g, w_m]`

Rules:

- Default `Q_c` can be block-diagonal but full blocks are allowed.
- `P` must remain full and can develop cross-covariance through EKF updates.

### 4.3 Discretization

For each interval `Δt` between time nodes:

- `F ≈ I + AΔt`
- `Q ≈ G Q_c Gᵀ Δt`

These are first-order approximations suitable for replay. The design must
allow upgrading to higher-order methods without changing interfaces.

---

## 5. Measurement models

### 5.1 IMU measurement (gyro)

Measurement in `{I}`:

- `z_ω = ω_raw`
- `R_ω = Σ_ω_raw`

Prediction:

- Let `R_BI` be IMU → body rotation from `T_BI`, and `R_IB = R_BIᵀ`.
- The estimated body angular rate is `ω_WB` expressed in `{B}`.
- Predicted IMU-frame angular rate:
  - `ω̂_I = R_IB * ω_WB + b_g`

Residual:

- `ν_ω = z_ω - ω̂_I`

### 5.2 IMU measurement (accel)

Measurement in `{I}`:

- `z_a = a_raw`
- `R_a = Σ_a_raw`

Convention:

`imu_raw.linear_acceleration` is treated as the raw accelerometer measurement
of specific force (i.e., it measures `a - g`, so at rest it measures approximately `-g`
in the sensor frame, up to calibration and noise).

- `a_WB ≈ 0`
- `f_B ≈ R_WB * (0 - g_W)`
- `z_a` measures `f_I` (up to calibration and noise)

Definitions:

- Let `R_BI` be IMU → body rotation from `T_BI`, and `R_IB = R_BIᵀ`.

Compute specific force:

- `a_WB = v̇_WB`
- `f_B = R_WB * (a_WB - g_W)`
- `f_I = R_IB * f_B`

Apply accel calibration model:

- `a_corr = A_a (a_raw - b_a)`
- Predicted raw accel:
  - `â_I = A_a^{-1} * f_I + b_a`

Residual:

- `ν_a = z_a - â_I`

### 5.3 Magnetometer measurement

Measurement in `{M}`:

- `z_m = m_raw`
- `R_m(t)` adaptive, initialized from `Σ_m_raw`

Definitions:

- Let `R_BM` be the mag → body rotation from `T_BM`.
- Then body → mag rotation is `R_MB = R_BMᵀ`.

Prediction:

- `m̂_M = R_MB * R_WB * m_W`

Residual:

- `ν_m = z_m - m̂_M`

### 5.4 Adaptive magnetometer covariance

Let `Ŝ = HPHᵀ` be the predicted innovation covariance without `R_m`.
Update `R_m` using:

- `R_m ← clamp_SPD( (1-α) R_m + α (ν νᵀ - Ŝ), R_min, R_max )`

Default values:

- `α = 0.01`
- `R_min = diag([1e-12, 1e-12, 1e-12])` tesla²
- `R_max = diag([2.5e-9, 2.5e-9, 2.5e-9])` tesla²
- `R_m0` from first valid `Σ_m_raw` else
  `diag([4e-10, 4e-10, 4e-10])` tesla²

---

## 6. Filter structure (EKF)

### 6.1 Predict step

- Input: current mean `x_k`, covariance `P_k`, time step `Δt`
- Output: predicted mean `x_{k+1|k}`, covariance `P_{k+1|k}`

Responsibilities:

- Integrate process model for the mean
- Compute `F` and `Q` for covariance propagation

### 6.2 Update step

Each measurement update is independent and uses the standard EKF update:

- `S = H P Hᵀ + R`
- `K = P Hᵀ S^{-1}`
- `δx = K ν`
- `x ← x ⊕ δx`
- `P ← (I - K H) P (I - K H)ᵀ + K R Kᵀ`

`⊕` denotes applying a perturbation to the mean state, including quaternion
and SE(3) updates.

### 6.3 Measurement ordering

At a single timestamp:

1. Apply priors (e.g., initial calibration) once
2. Gyro update (from IMU packet, if present)
3. Accel update (from IMU packet, if present)
4. Mag update (from magnetometer sample, if present)

Ordering must be deterministic and independent of arrival order.

#### Same-timestamp attachment policy

If an IMU packet and a magnetometer sample share the same `t_meas_ns`, they attach
to the same node and are both applied using the fixed ordering above.

If a second measurement of the same type arrives for an existing node
(same `t_meas_ns`), it must be rejected (or replace-with-diagnostics) according to
an explicit policy. Default: reject as a duplicate and emit diagnostics.

---

## 7. Time handling and replay

Implement a fixed-lag deterministic replay system.

### 7.1 Buffer model

Maintain a time-ordered ring buffer of nodes. Each node stores:

- mean state and covariance at that time
- attached measurements and metadata

Nodes are keyed by timestamp. Messages with the same timestamp attach to the
same node.

Rules:

- A node may contain **multiple measurement types** at the same timestamp
  (e.g., an IMU packet and a magnetometer sample with identical `t_meas_ns`).
- By contract, each node stores at most one instance per measurement type:
  - `imu_pkt` slot: ≤ 1 IMU packet per `t_meas_ns`
  - `mag` slot: ≤ 1 magnetometer sample per `t_meas_ns`
- If a message arrives for a type slot that is already occupied at that `t_meas_ns`,
  it is treated as a duplicate per Section 6.3 (default: reject + diagnostics).

### 7.2 Replay rules

On insertion of a message:

1. Validate timestamp
2. Reject if older than `t_filter_ns - t_buffer_sec` * 1e9
3. Attach to node (create if missing); if the node already exists, attach the
   measurement into its type slot (IMU packet or mag sample). Reject duplicates
   per Section 6.3.
4. If inserted into the past, replay forward to the frontier

Replay must be deterministic and not depend on arrival order.

### 7.3 Frontier publish

Only publish the estimate when the frontier time advances. Out-of-order
updates do not republish.

---

## 8. ROS integration contract

The ROS layer should be thin and only translate to/from messages, handle
parameters, QoS, and logging. All math and state logic must reside in
`oasis_control/oasis_control/localization/ahrs/`.

### 8.1 Inputs

- `imu_raw` (`sensor_msgs/Imu`)
- `imu_calibration` (`oasis_msgs/ImuCalibration`)
- `magnetic_field` (`sensor_msgs/MagneticField`)

### 8.2 Outputs

- TF (`world -> odom`, `odom -> base_link`)
- `nav_msgs/Odometry` (`ahrs/odom`, `ahrs/world_odom`)
- `oasis_msgs/AhrsState`
- `oasis_msgs/AhrsDiagnostics`
- `oasis_msgs/EkfUpdateReport` for each measurement update

### 8.3 Frames and TF contract (world / odom)

The AHRS publishes the standard TF chain:

`world -> odom -> base_link`

Policy (continuous, no teleports):

- The AHRS does not apply global corrections. All outputs must be continuous in
  time (no discontinuities/teleports).
- The `odom` frame is the locally drifting frame used for smooth motion.

Recommended implementation contract:

- Publish `TF: world -> odom` as identity for the entire session:
  - `T_WO := I`
- Publish `TF: odom -> base_link` as the AHRS estimate:
  - `T_OB := T_WB` (numerically equivalent under `T_WO := I`)

Relationship: `T_WB = T_WO ∘ T_OB`, so under `T_WO := I`, `T_WB == T_OB`.

This keeps the TF tree compatible with a future localization system that may
later publish a non-identity `T_WO` (global correction), without changing
`odom -> base_link`.

---

## 9. Component specs

This section defines the main classes. Each class should be implemented in
small steps, with tests before integration.

### 9.1 `state/ahrs_state.py`

**Responsibilities**:

- Owns the mean state values
- Provides serialization to plain dict/struct for testing

**Key methods**:

- `copy()`
- `apply_error_state(delta)`

### 9.2 `state/error_state.py`

**Responsibilities**:

- Defines the error-state vector layout
- Provides mapping between index ranges and named blocks

### 9.3 `state/covariance.py`

**Responsibilities**:

- Stores `P` and provides block access
- Ensures symmetry (e.g., force `P = 0.5 * (P + Pᵀ)` after updates)

### 9.4 `models/process_model.py`

**Responsibilities**:

- Integrates mean state for `Δt`
- Computes continuous-time Jacobians `A`, `G`
- Computes discrete `F`, `Q`

**Math**:

- Use Section 4 equations
- Document every parameter in code with units and meaning

### 9.5 `models/imu_model.py`

**Responsibilities**:

- Implements gyro and accel measurement functions
- Computes residuals, Jacobians, and expected measurements

**Math**:

- Use Section 5.1 and 5.2

### 9.6 `models/mag_model.py`

**Responsibilities**:

- Implements magnetometer measurement function
- Computes residuals, Jacobians, and expected measurements

**Math**:

- Use Section 5.3

### 9.7 `models/noise_adaptation.py`

**Responsibilities**:

- Maintains adaptive `R_m` using covariance matching
- Applies `clamp_SPD` to bound eigenvalues

**Math**:

- Use Section 5.4

### 9.8 `filter/ekf.py`

**Responsibilities**:

- Orchestrates predict and update
- Owns state and covariance
- Exposes a stable API: `predict(dt)`, `update_gyro(...)`,
  `update_accel(...)`, `update_mag(...)`

### 9.9 `time/replay_engine.py`

**Responsibilities**:

- Manages buffer and deterministic replay
- Provides `insert_measurement(...)` and `advance_frontier(...)`

### 9.10 `nodes/ahrs/ahrs_node.py`

**Responsibilities**:

- Subscribes to ROS topics
- Converts incoming messages into core measurements
- Publishes state and diagnostics

---

## 10. Implementation order (recommended)

1. Math primitives (`quat`, `se3`, `linalg`, `stats`)
2. State definitions and mappings
3. Process model
4. IMU measurement model
5. Magnetometer model + adaptive noise
6. EKF core
7. Time buffer + replay engine
8. ROS integration

---

## 11. Required documentation in code

- Every public struct/enum must include field docs with units and meaning
- Every constant must have a comment describing its derivation or units
- Comments should wrap at 80 chars

---

## 12. Non-goals

- No GPS or global corrections
- No external localization fusion in this component
- No direct dependence on ROS inside `oasis_control/oasis_control/localization/ahrs/`
  (ROS types and wiring live only under `oasis_control/oasis_control/nodes/ahrs/`).
