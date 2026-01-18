# AHRS Design Document

This document defines the **state-space model** and **time-handling policy** for an AHRS that uses:

- a continuous-time **process model**: kinematic evolution + slow parameter drift (priors), and
- discrete-time **measurement models**:
  - IMU gyro + accel measurements (`imu_raw`) and
  - magnetometer measurements (`magnetic_field`),

all applied on a common time axis with deterministic fixed-lag replay.

---

## World / Odom frame contract

The AHRS publishes the standard TF chain:

`world -> odom -> base_link`

Policy (no teleports, no global correction):

- `world` is initialized at startup to the origin (continuous).
- `odom` is continuous and **drifts locally**.
- There are no discontinuities because no global constraints exist.

Recommended implementation contract:

- Publish `TF: world -> odom` as identity for the entire session:
  - `T_WO := I`
- Publish `TF: odom -> base_link` as the AHRS estimate:
  - `T_OB := T_WB` (since `world == odom` under the identity `T_WO`)

This keeps the TF tree compatible with a future localization EKF that may later produce non-identity `T_WO`.

---

## 1. ROS Inputs

### 1.1 Streams

- Single `message_filters` combination (ExactTime) producing an **IMU packet**:
  - `imu_raw` (`sensor_msgs/Imu`): inertial sample stream (time-stamped)
  - `imu_calibration` (`oasis_msgs/ImuCalibration`): calibration priors + uncertainty
    - **Semantic rule:** used **only once** to initialize in-state calibration parameters (initial prior).
    - After initialization, `imu_calibration` contents are ignored (except diagnostics / consistency checks).
- `magnetic_field` (`sensor_msgs/MagneticField`): magnetometer samples (time-stamped)

---

### 1.2 `imu_raw` — `sensor_msgs/Imu`

Used fields:

- `header.stamp` (`t_meas`)
- `header.frame_id` → IMU frame `{I}`
- `angular_velocity` `ω_raw` and **full** `angular_velocity_covariance` `Σ_ω_raw` (3×3)
- `linear_acceleration` `a_raw` and **full** `linear_acceleration_covariance` `Σ_a_raw` (3×3)

Ignored:

- `orientation`, `orientation_covariance`

Requirements:

- Covariances are interpreted as **per-sample measurement noise**.
- `ω_raw`, `a_raw` are expressed in `{I}`.
- Do not diagonalize any provided covariance.

---

### 1.3 `imu_calibration` — `oasis_msgs/ImuCalibration`

Role: **one-shot initialization prior** for systematic parameters that live in the shared state.

Validity:

- If `valid == false`, ignore.

Frame:

- `header.frame_id` must match `imu_raw.header.frame_id` (calibration is defined in `{I}`).

Models (parameter meaning):

- Accel: `a_corr = A_a (a_raw − b_a)` where `b_a, A_a` are **in-state**
- Gyro: `ω_corr = ω_raw − b_g` where `b_g` is **in-state**

Requirements:

- Consume **only while uninitialized**:
  - The **first** valid calibration observed is applied as an initial prior on `(b_a, A_a, b_g)` with its covariance.
  - After initialization, ignore subsequent calibration messages (except diagnostics).
- Calibration uncertainty is **systematic parameter uncertainty**, not per-sample noise.
- Always use full covariance matrices when provided (do not diagonalize).

---

### 1.4 `magnetic_field` — `sensor_msgs/MagneticField`

Used fields:

- `header.stamp` (`t_meas`)
- `header.frame_id` → mag frame `{M}`
- `magnetic_field` `m_raw`
- **full** `magnetic_field_covariance` `Σ_m_raw` (3×3)

Measurement convention:

- Form magnetometer residual in `{M}` (do not rotate the raw measurement into `{B}`):
  - `z := m_raw` in `{M}`
  - `z_hat := m̂_M` in `{M}`
  - `ν := z - z_hat` in `{M}`

Noise interpretation:

- `Σ_m_raw` is the driver’s estimate of measurement noise covariance and is used as a prior / input to the
  internally maintained `R_m(t)` (but not treated as per-sample truth).
- Do not diagonalize any provided covariance.

Adaptive noise policy (covariance matching):

- Initialize `R_m0` from first valid `Σ_m_raw`, else default:
  - `R_m0 = diag([4e-10, 4e-10, 4e-10])` (tesla²) (20 µT RMS per axis)
- Maintain adaptive `R_m(t)` bounded SPD:
  - `R_min = diag([1e-12, 1e-12, 1e-12])` (tesla²)
  - `R_max = diag([2.5e-9, 2.5e-9, 2.5e-9])` (tesla²)
- Update:
  - Let innovation be `ν_k`, predicted innovation covariance excluding R be `Ŝ_k = HPHᵀ`
  - `R_m ← clamp_SPD( (1-α) R_m + α (ν_k ν_kᵀ - Ŝ_k), R_min, R_max )`
  - default `α = 0.01`

---

## 2. Frames and Extrinsics

Frames:

- `{W}`: world frame, ROS frame_id = `world`
- `{O}`: odom frame, ROS frame_id = `odom`
- `{B}`: body frame, ROS child_frame_id = `base_link`
- `{I}`: IMU measurement frame (`imu_raw.header.frame_id`)
- `{M}`: magnetometer frame (`magnetic_field.header.frame_id`)

Estimated extrinsics (in-state):

- `T_BI` (IMU → body)
- `T_BM` (mag → body)

Parameterization:

- Poses in SE(3): quaternion rotation + 3-vector translation
- Uncertainty: 6D tangent (δρ, δθ) for SE(3) quantities

Initialization priors (broad but finite; full covariances allowed and recommended):

- `T_BI` prior:
  - mean: identity
  - rotation stddev: `π` rad per axis (tangent)
  - translation stddev: `1.0 m` per axis
- `T_BM` prior:
  - mean: identity
  - rotation stddev: `π` rad per axis (tangent)
  - translation stddev: `1.0 m` per axis

---

## 3. State, Process Model, and Measurement Models

This AHRS is a **single optimizer** over a **shared state** with multiple models.

### 3.1 State (full model)

The AHRS maintains a full navigation + calibration + extrinsic state:

Navigation (in `{W}` unless noted):

- Position: `p_WB ∈ ℝ³`
- Velocity: `v_WB ∈ ℝ³`
- Attitude: `q_WB` (unit quaternion, world → body rotation)

IMU systematic parameters:

- Gyro bias: `b_g ∈ ℝ³`
- Accel bias: `b_a ∈ ℝ³`
- Accel scale/misalignment: `A_a ∈ ℝ^{3×3}` (full matrix)

Extrinsics (sensor-to-body in SE(3)):

- `T_BI ∈ SE(3)` (IMU → body)
- `T_BM ∈ SE(3)` (mag → body)

Environment / reference vectors (kept in-state, full covariance):

- Gravity vector in world: `g_W ∈ ℝ³`
- Earth magnetic field vector in world: `m_W ∈ ℝ³`

Full covariance policy:

- The AHRS maintains a single covariance `P` over its internal error-state.
- Do not diagonalize:
  - any incoming sensor covariance (`Σ_ω_raw`, `Σ_a_raw`, `Σ_m_raw`),
  - any priors / initialization covariances,
  - or internal state covariance blocks.

---

### 3.2 Error-state definition (for covariance)

The AHRS uses an error-state EKF.

Representative error coordinates:

- `δp, δv ∈ ℝ³`
- `δθ ∈ ℝ³` small-angle attitude error s.t. `q_WB ≈ Exp(δθ) ⊗ q̂_WB`
- `δb_g, δb_a ∈ ℝ³`
- `δA_a ∈ ℝ⁹` (row-major perturbation of `A_a`) or an equivalent minimal parameterization
- `δξ_BI ∈ ℝ⁶`, `δξ_BM ∈ ℝ⁶` (SE(3) tangent: translation + rotation)
- `δg_W, δm_W ∈ ℝ³`

`P` is the covariance over the stacked error-state vector above (full matrix).

---

### 3.3 Continuous-time process model (kinematic prior + slow drift)

The process model provides time evolution between measurements. Sensor data does **not** drive propagation in this option;
IMU and mag are measurement updates.

Define continuous-time dynamics:

Navigation kinematics:

- `ṗ_WB = v_WB`
- `v̇_WB = g_W + w_v`
- `q̇_WB = 1/2 * Ω(ω_WB) * q_WB`

where:
- `ω_WB` is the body angular rate (world → body) treated as **latent** (not directly measured in the process model),
  modeled as:
  - `ω_WB = w_ω`
- `w_v`, `w_ω` are zero-mean white noises (continuous-time) representing smoothness priors.

Systematic parameter drift (random walks):

- `ḃ_g = w_bg`
- `ḃ_a = w_ba`
- `Ȧ_a = W_A` (element-wise random walk; `vec(Ȧ_a) = w_A`)
- `Ṫ_BI = W_BI` (SE(3) random walk in tangent; `δξ̇_BI = w_BI`)
- `Ṫ_BM = W_BM` (SE(3) random walk in tangent; `δξ̇_BM = w_BM`)

Reference vector drift (slow):

- `ġ_W = w_g`
- `ṁ_W = w_m`

Notes:

- This process is intentionally **weak**: it encodes “motion is smooth” and “parameters drift slowly”.
- The IMU measurements will strongly constrain the state at high rate.

Process noise covariance:

- Continuous-time noise intensities:
  - `Q_c = cov([w_v, w_ω, w_bg, w_ba, w_A, w_BI, w_BM, w_g, w_m])`
- `Q_c` may be block-diagonal by default, but the **state covariance P is full** and will develop cross-terms through
  Jacobians and updates.
- If you have known cross-correlations (e.g., coupled accel axes), you may encode them in `Q_c` (full blocks permitted).

Discretization:

- Over interval `Δt`, compute discrete `F_k` and `Q_k` using a standard EKF discretization
  (e.g., first-order: `Q_k ≈ G Q_c Gᵀ Δt`, with `F_k ≈ I + AΔt`),
  keeping full matrices.

---

### 3.4 IMU measurement model (discrete-time)

At each `imu_raw` timestamp `t_k`, treat IMU outputs as measurements.

#### 3.4.1 Frame handling for IMU

- IMU raw signals are in `{I}`.
- Use the estimated extrinsic `T_BI` to relate `{I}` and `{B}`:
  - Let `R_BI` be the rotation from IMU to body from `T_BI`.
  - Then body-to-IMU rotation is `R_IB = R_BIᵀ`.

#### 3.4.2 Gyro measurement

Measurement:

- `z_ω := ω_raw` in `{I}` with covariance `R_ω := Σ_ω_raw` (full 3×3)

Prediction:

- Predict IMU-frame angular rate as:
  - `ω̂_I = R_IB * ω_WB + b_g`

Residual:

- `ν_ω = z_ω - ω̂_I` (in `{I}`)

#### 3.4.3 Accelerometer measurement

Measurement:

- `z_a := a_raw` in `{I}` with covariance `R_a := Σ_a_raw` (full 3×3)

Specific-force prediction:

- Compute world-frame acceleration of the body origin from kinematics:
  - `a_WB := v̇_WB` (from process state; in the mean model, `v̇_WB ≈ g_W`, but this is refined by measurements)
- Specific force in body frame is:
  - `f_B = R_BW * (a_WB - g_W)` where `R_BW` is world → body rotation from `q_WB`
- Convert to IMU frame:
  - `f_I = R_IB * f_B`

Apply accel systematic model (as defined by calibration semantics):

- The measurement model uses:
  - `a_corr = A_a (a_raw − b_a)`
- Equivalently, the predicted raw measurement is:
  - `â_raw = A_a^{-1} * f_I + b_a`

Prediction:

- `â_I = A_a^{-1} * f_I + b_a`

Residual:

- `ν_a = z_a - â_I` (in `{I}`)

Notes:

- The above uses a full 3×3 `A_a`. If `A_a` is near-singular, the state/prior must prevent it (bounded parameterization
  recommended). The covariance remains full regardless of parameterization.
- `R_a` is always taken as the provided full covariance (or a configured fallback if invalid).

---

### 3.5 Magnetometer measurement model (discrete-time)

At each `magnetic_field` timestamp `t_k`:

- Measurement: `z_m := m_raw` in `{M}`
- Covariance input: `Σ_m_raw` (full 3×3), used to initialize / inform the adaptive `R_m(t)`.

Prediction (in `{M}`; residual always formed in `{M}`):

- Let `R_BM` be the rotation of `T_BM` (mag → body), so body→mag is `R_MB = R_BMᵀ`.
- Let `R_BW` be world → body rotation from `q_WB`.
- Predict:
  - `m̂_M = R_MB * R_BW * m_W`

Residual:

- `ν_m = z_m - m̂_M` in `{M}`

Measurement noise used:

- `R_m(t)` (adaptive, bounded SPD; full 3×3)

---

## 4. Time Axis, Buffer, and Deterministic Replay (Fixed-lag)

This AHRS uses the same deterministic fixed-lag design as the EKF spec.

### 4.1 Time definitions

- `t_meas`: `header.stamp` from the incoming message (authoritative event time)
- `t_now`: receipt time (diagnostics only)
- `t_filter`: max `t_meas` among **accepted** messages (frontier)

Event-driven: each accepted message attaches at `t_meas`. Out-of-order messages are supported via fixed-lag replay.

### 4.2 Message roles

- **Measurement updates (high-rate):** `imu_raw` (gyro + accel updates)
- **Initialization prior (one-shot):** `imu_calibration`
- **Measurement updates:** `magnetic_field`
- **Process model:** runs continuously between time nodes as the kinematic prior and slow parameter drift.

### 4.3 IMU packet synchronization contract

The AHRS consumes IMU as a synchronized pair:

- `imu_pkt = (imu_raw, imu_calibration)` produced by `message_filters` using **ExactTime**.

Timestamp policy:

- Packet time: `t_meas := imu_raw.header.stamp`
- Require `imu_calibration.header.stamp == imu_raw.header.stamp`; reject packet if not.

Semantic policy:

- `imu_calibration` is used **only before initialization** to set priors on `(b_a, A_a, b_g)` with full covariance.
- After initialization, `imu_calibration` is ignored (except diagnostics).
- `imu_raw` always produces measurement updates (gyro + accel) when accepted.

### 4.4 Buffer model

Maintain a time-ordered ring buffer of time nodes over fixed lag `T_buffer_sec`.

Each node at `t_k` stores:

- state mean `x_k`, covariance `P_k`
- messages attached at `t_k` (IMU packet, mag)
- sufficient metadata to re-run deterministic replay

Node rules:

- One node per distinct timestamp.
- Multiple messages may attach to the same node.
- Evict nodes older than `t_filter - T_buffer_sec` (with diagnostics).

### 4.5 Propagation convention

Between time nodes, propagate with the continuous-time process model (Section 3.3), discretized over each interval.

Strict coverage rule:

- For deterministic replay, the AHRS must be able to propagate across any interval used in replay.
- If there is a timestamp gap larger than `Δt_imu_max` between consecutive nodes required to cover replay,
  reject replay across that gap and emit diagnostics.

### 4.6 Insertion + replay (deterministic)

On message arrival:

1. Validate timestamp
   - reject missing/invalid
   - reject `t_meas > t_now + ε_wall_future`

2. Fixed-lag window
   - if `t_filter` exists and `t_meas < t_filter - T_buffer_sec`, reject as too old

3. Attach
   - insert/attach message to node `t_meas` (create node if needed)

4. Update frontier / replay
   - if `t_meas > t_filter`: advance `t_filter` and propagate forward
   - else: apply update at `t_meas`, then replay forward to `t_filter`

Per-node application order (stable):

1. Apply any priors/parameter constraints at `t_k` (including one-shot calibration prior if first valid)
2. Apply measurement updates at `t_k` in stable tie-break order:
   - lexicographic `(message_type, topic, frame_id)`
   - Within IMU: apply gyro update then accel update (fixed order)

Replay must never depend on arrival order.

### 4.7 Drop / reset rules

Reject and emit diagnostics if:

- missing timestamp
- required covariance contains NaN/Inf
- message outside fixed-lag window
- propagation coverage insufficient (`Δt_imu_max` violated)
- clock discontinuity detected: reset filter + buffer if `|t_meas - t_filter| > Δt_clock_jump_max`
  - Reset effects (explicit):
    - clear the time buffer and set `t_filter` to unset
    - set `initialized = false`
    - allow consumption of the next valid `imu_calibration` as a new initialization prior
    - reset adaptive `R_m(t)` to `R_m0`

---

### 4.8 Output publish trigger (explicit)

The AHRS publishes "current estimate" outputs **only when the frontier advances**.

- Let `t_filter_prev` be the frontier before processing an incoming message.
- After processing a message:
  - If `t_meas > t_filter_prev` (frontier advance), the filter propagates to `t_meas`, applies updates at `t_meas`,
    sets `t_filter := t_meas`, and **publishes**:
      - TF (`world -> odom`, `odom -> base_link`) stamped `t_filter`
      - odometry outputs stamped `t_filter`
      - `oasis_msgs/AhrsState` stamped `t_filter`
      - `oasis_msgs/AhrsDiagnostics` stamped `t_filter`
  - If `t_meas <= t_filter_prev` (out-of-order insertion), the filter may replay internally to maintain consistency at
    `t_filter`, but **does not republish** TF/odometry/state/diagnostics (to avoid emitting past timestamps).

Per-measurement update reports (`oasis_msgs/EkfUpdateReport`) are published at the measurement timestamp `t_meas` for
every accepted/rejected update, regardless of whether the frontier advanced.

---

## 5. Outputs (poses + covariances + measurement stats)

The AHRS publishes:

1. state estimates in ROS-standard frames (`world`, `odom`, `base_link`)
2. full covariances for those estimates (not diagonalized)
3. per-measurement innovation / gating statistics (so “all measurements” have covariances too)

### 5.1 TF

- Broadcast `TF: odom -> base_link` from `T_OB` (`stamp = t_filter`)
- Broadcast `TF: world -> odom` as identity (`stamp = t_filter`)

TF is published on `/tf` (standard tf2 broadcaster).

### 5.2 Odometry messages

**(A) `nav_msgs/Odometry` in `odom`**

- Topic: `ahrs/odom`
- `header.stamp = t_filter`
- `header.frame_id = "odom"`
- `child_frame_id = "base_link"`
- Pose: `T_OB` (translation from `p_WB`, rotation from `q_WB`)
- Twist: derived from state (e.g., linear from `v_WB`, angular from `ω_WB` estimate as implied by IMU updates)
- Covariance: derived from `P` (full covariance propagation, mapped into ROS odom convention)

**(B) `nav_msgs/Odometry` in `world`**

- Topic: `ahrs/world_odom`
- Same values as (A) under `T_WO = I` contract; covariances derived from the same `P`.

### 5.3 Extrinsic outputs

- `geometry_msgs/PoseWithCovarianceStamped`:
  - Topic: `ahrs/extrinsics/t_bi` for `T_BI`
  - Topic: `ahrs/extrinsics/t_bm` for `T_BM`
  - `header.stamp = t_filter`
  - `header.frame_id = "base_link"`
  - Covariance derived from `P` (full 6×6, not diagonalized)
  - Publish: **on frontier advance** (Section 4.8)

### 5.4 Covariance transformation rules

The AHRS maintains a single covariance `P` over its internal error-state. Published covariances are obtained by:

`Σ_y = J * P * Jᵀ`, with `J = ∂g/∂x` evaluated at `x̂`.

For SE(3) pose errors in tangent (δρ, δθ), use the adjoint:

`Σ_A = Ad_{T_AB} * Σ_B * Ad_{T_AB}ᵀ`

All covariance matrices are full and must remain symmetric positive definite (or PSD as appropriate).

### 5.5 Per-measurement reports

Publish one report entry for every accepted/rejected update:

- `ahrs/updates/accel`
- `ahrs/updates/gyro`
- `ahrs/updates/mag`

Contents (recommended):

- `header.stamp = t_meas`
- `z`, `z_hat`, innovation `ν`
- measurement noise used `R` (full)
- predicted innovation covariance excluding R: `Ŝ = HPHᵀ`
- total innovation covariance `S = Ŝ + R`
- Mahalanobis distance `d² = νᵀ S^{-1} ν`
- gating threshold + accept/reject flag

Each per-measurement topic publishes `oasis_msgs/EkfUpdateReport` stamped at `t_meas` for every accepted/rejected update.

### 5.6 `oasis_msgs/AhrsState` (full state + full covariance)

- Topic: `ahrs/state`
- Publish: **on frontier advance** (Section 4.8)
- `header.stamp = t_filter`
- Contents: full mean state (navigation + calibration + extrinsics + `g_W`, `m_W`) and full error-state covariance `P`
  with explicit ordering (`error_state_names`). No diagonalization.

### 5.7 `oasis_msgs/AhrsDiagnostics` (buffer + drop/replay stats)

- Topic: `ahrs/diag`
- Publish: **on frontier advance** (Section 4.8)
- `header.stamp = t_filter`
- Contents: buffer size/span, replay flag, and monotonic drop/reset counters.

---

## 6. Config Parameters

Frames:

- `world_frame_id` (default `"world"`)
- `odom_frame_id` (default `"odom"`)
- `body_frame_id` (default `"base_link"`)

Time / buffering:

- `T_buffer_sec`
- `ε_wall_future`
- `Δt_clock_jump_max`
- `Δt_imu_max`

Process noise intensities (continuous-time):

- `Q_v` (for `w_v`)
- `Q_ω` (for `w_ω`)
- `Q_bg`, `Q_ba`
- `Q_A` (for `vec(A_a)` drift)
- `Q_BI`, `Q_BM` (for extrinsic drift in SE(3) tangent)
- `Q_g`, `Q_m` (for `g_W`, `m_W` drift)

Magnetometer adaptive noise:

- `α`
- `R_min`, `R_max`
- default `R_m0` (if no valid `Σ_m_raw` observed)
