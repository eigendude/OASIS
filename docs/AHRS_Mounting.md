# AHRS Mounting Calibration

This document specifies **AHRS Mounting Calibration**: a ROS 2 node that estimates only the **mounting rotations** for the IMU and magnetometer relative to the robot body (`base_link`), publishes them as TF with **zero translation**, and **saves** the calibration for use by the AHRS node.

This exists because the AHRS EKF can be poorly conditioned at boot (especially when stationary) if the IMU/mag-to-body rotations are wrong or unknown. The mount-calibration node learns those rotations first so the AHRS node starts from a physically consistent TF chain.

---

## 1. Goals and non-goals

### 1.1 Goals

- Estimate mounting **rotations**:
  - `R_BI`: IMU frame `{I}` → body `{B}` (`base_link`)
  - `R_BM`: magnetometer frame `{M}` → body `{B}` (`base_link`)
- Publish those as TF (with **zero translation**) while calibrating.
- Persist the result to disk in a deterministic format (YAML/JSON) that the AHRS node can load.
- Work with **limited motion** (e.g., can’t invert fully; can tilt up to ~80°).

### 1.2 Non-goals

- This node is **not** a navigation filter. It does not estimate `p_WB`, `v_WB`, or drift-free world pose.
- It does **not** permanently calibrate accel `A_a` / biases for the AHRS state. (It may estimate nuisance parameters internally to make the mount solve well, but it only _outputs_ mount TF.)
- It does **not** do hard/soft-iron magnetometer calibration beyond what is required to make a stable mount estimate (optional nuisance).

---

## 2. ROS Inputs

### 2.1 Streams

- Single `message_filters` combination (ExactTime) producing an **IMU packet**:
  - `imu_raw` (`sensor_msgs/Imu`): inertial sample stream (time-stamped)
  - `imu_calibration` (`oasis_msgs/ImuCalibration`): calibration priors + uncertainty
    - **Deterministic contract:** IMU processing requires ExactTime pairing of
      `(imu_raw, imu_calibration)` at identical `t_meas_ns`. Any `imu_raw`
      without a matching `imu_calibration` at the exact same timestamp MUST be
      rejected/dropped. This is intentional even if calibration is static.
    - `imu_calibration` will not change over the time taken to calibrate mounting parameters
    - **Semantic rule:** used **only once** to initialize in-state calibration
      parameters (initial prior).
    - After initialization, `imu_calibration` does NOT modify in-state
      parameters.
- `magnetic_field` (`sensor_msgs/MagneticField`): magnetometer samples (time-stamped)

---

### 2.2 `imu_raw` — `sensor_msgs/Imu`

Used fields:

- `header.stamp` (`t_meas_ns`)
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

### 2.3 `imu_calibration` — `oasis_msgs/ImuCalibration`

Role: **one-shot initialization prior** for systematic parameters that live in
the shared state.

Validity:

- If `valid == false`, ignore.

Frame:

- `header.frame_id` must match `imu_raw.header.frame_id` (calibration is defined in `{I}`).

Models (parameter meaning):

- Accel: `a_corr = A_a (a_raw − b_a)` where `b_a, A_a` are **in-state**
- Gyro: `ω_corr = ω_raw − b_g` where `b_g` is **in-state**

Requirements:

- IMU processing requires ExactTime pairing of `(imu_raw, imu_calibration)` at
  identical `t_meas_ns`. Any `imu_raw` without a matching `imu_calibration` at
  the exact same timestamp MUST be rejected/dropped, even when calibration is
  static and unchanged.
- Consume **only while uninitialized**:
  - The **first** valid calibration observed is applied as an initial prior on
    `(b_a, A_a, b_g)` with its covariance.
  - After initialization, do not modify in-state parameters from subsequent
    calibration messages. Still record diagnostics/consistency checks and allow
    a future policy hook for re-init or reset on calibration change (TODO,
    policy only).
- Calibration uncertainty is **systematic parameter uncertainty**, not per-sample noise.
- Always use full covariance matrices when provided (do not diagonalize).

---

### 2.4 `magnetic_field` — `sensor_msgs/MagneticField`

Used fields:

- `header.stamp` (`t_meas_ns`)
- `header.frame_id` → mag frame `{M}`
- `magnetic_field` `m_raw`
- **full** `magnetic_field_covariance` `Σ_m_raw` (3×3, tesla²)

Measurement convention (mount solve):

- The mount solver uses **direction-only** mag factors (consistent with §6):
  - `m̂_M := normalize(m_raw)` in `{M}`
  - predicted `m̂̂_M` is also a unit vector in `{M}`
  - direction residual: `r_m := m̂_M × m̂̂_M` (unitless; ≈ radians for small errors)

Noise interpretation:

- The solver maintains a direction-residual noise model `R_m(t)` with units **unitless²** (≈ rad²).
- The driver-provided `Σ_m_raw` (tesla²) is used only as an _optional prior_ by converting it to a direction covariance:
  - Let `u := normalize(m_raw)` and `s := ||m_raw||`.
  - Require `s > s_min` for covariance conversion (default `s_min = 1e-6` T) to avoid numerical blow-up.
  - Jacobian of normalization: `J := (I - u uᵀ) / s`
  - Direction covariance prior: `Σ_dir ≈ J Σ_m_raw Jᵀ` (unitless²; approximately angular variance on the unit sphere)

- If `Σ_m_raw` is missing/invalid or `s` is too small, fall back to defaults.

Adaptive noise policy (covariance matching on direction residuals):

- Initialize:
  - `R_m0 := Σ_dir` if available else `diag([1e-3, 1e-3, 1e-3])` (unitless²)
- Maintain adaptive `R_m(t)` bounded SPD:
  - `R_min = diag([1e-5, 1e-5, 1e-5])`
  - `R_max = diag([1e-1, 1e-1, 1e-1])`
- Update:
  - Let innovation be `r_m,k`, predicted innovation covariance excluding `R_m` be `Ŝ_k = HPHᵀ`
  - `R_m ← clamp_SPD( (1-α) R_m + α (r_m,k r_m,kᵀ - Ŝ_k), R_min, R_max )`
  - default `α = 0.01`

---

## 3. Frames and TF contract

Frames:

- `{W}`: world (`world`) — only used internally as an abstract reference during calibration
- `{B}`: body (`base_link`)
- `{I}`: IMU sensor frame (from `imu_raw.header.frame_id`, e.g., `imu_link`)
- `{M}`: magnetometer sensor frame (from `magnetic_field.header.frame_id`, e.g., `mag_link`)

Estimated mount rotations (co-located sensors):

- `R_BI` : rotation IMU frame `{I}` → body `{B}`
- `R_BM` : rotation mag frame `{M}` → body `{B}`

For TF publishing, translations are fixed to zero:

- `p_BI := [0, 0, 0]`
- `p_BM := [0, 0, 0]`

Therefore the published transforms are:

- `T_BI = (R_BI, 0)`
- `T_BM = (R_BM, 0)`

### 3.1 TF direction convention

We use TF2 parent/child semantics.

- Published TF:
  - parent: `base_link` (`{B}`)
  - child: IMU frame `{I}` and mag frame `{M}`

Therefore:

- `T_BI` maps IMU-frame coordinates into body-frame coordinates:
  - `v_B = R_BI v_I`
  - `p_B = R_BI p_I + p_BI`
- `T_BM` maps mag-frame coordinates into body-frame coordinates:
  - `v_B = R_BM v_M`
  - `p_B = R_BM p_M + p_BM`

With co-location, `p_BI = p_BM = 0`, so translation terms are identically zero.

Quaternions in persisted files MUST declare their ordering explicitly (e.g. `quaternion_wxyz`).

### 3.2 Co-location policy (no translation)

Mount translation is **not estimated** and is **not parameterized**.

Contract:

- The IMU and magnetometer are treated as **co-located** for mounting calibration.
- Translations are fixed constants:
  - `p_BI := [0, 0, 0]` meters
  - `p_BM := [0, 0, 0]` meters
- The optimizer SHALL NOT include `p_BI` or `p_BM` as decision variables.
- No translation priors, translation regularization, or translation constraints SHALL appear in the problem.

Implications:

- Published TF transforms SHALL use zero translation for both sensors.
- Persisted calibration SHALL NOT include translation fields

---

## 4. ROS Interface

### 4.1 Node name

- Node: `ahrs_mounting`

### 4.2 Inputs (subscriptions)

#### 4.2.1 IMU

- Topic: `imu_raw` (`sensor_msgs/Imu`)
- Used fields:
  - `header.stamp`
  - `header.frame_id` → IMU frame `{I}`
  - `angular_velocity` and `angular_velocity_covariance` (3×3 full)
  - `linear_acceleration` and `linear_acceleration_covariance` (3×3 full)
- Ignored:
  - `orientation`, `orientation_covariance`

#### 4.2.2 IMU calibration prior

- Topic: `imu_calibration` (`oasis_msgs/ImuCalibration`)
- Role: **one-shot initialization prior** for in-state nuisance parameters (per §2.3)
- Contract:
  - MUST be ExactTime paired with `imu_raw` at identical `t_meas_ns` (deterministic; `imu_raw` without a match is dropped) even after initialization.
  - Content is consumed **only while uninitialized** to set the initial prior on `(b_a, A_a, b_g)`.
  - After initialization, calibration messages are still paired for determinism/diagnostics, but **do not update** parameters.

#### 4.2.3 Magnetometer

- Topic: `magnetic_field` (`sensor_msgs/MagneticField`)
- Used fields:
  - `header.stamp`
  - `header.frame_id` → mag frame `{M}`
  - `magnetic_field` and `magnetic_field_covariance` (3×3 full)

### 4.3 Outputs (publishers)

#### 4.3.1 TF output

- Publish `base_link -> imu_link` and `base_link -> mag_link` as TF2 transforms.
- While the solution is changing, publish to `/tf` at the optimizer update rate.
- Once stable enough, publish to `/tf_static` (latched). If the saved calibration file changes later, **republish the full static transforms** on `/tf_static` (do not stream `/tf_static` continuously).

“Stable enough” is when both:

- `||Δθ_BI|| < stable_rot_thresh_rad` and `||Δθ_BM|| < stable_rot_thresh_rad` over the last `stable_window_sec`
- and solver reports `need_more_tilt == false` (and `need_more_yaw == false` when mag is being used)

#### 4.3.2 Result topics

**Primary structured result**

- Topic: `ahrs_mount` (`oasis_msgs/AhrsMount`)
- Publish rate: on optimizer update (and at least once per `save_period_sec`)
- Contents:
  - `header.stamp`: time of the current estimate
  - Frames:
    - `base_frame` (`base_link`)
    - `imu_frame` (from `imu_raw.header.frame_id`)
    - `mag_frame` (from `magnetic_field.header.frame_id`)
  - Mount rotations (co-located sensors; no translation):
    - `R_BI` (rotation only)
    - `R_BM` (rotation only)
  - Covariances:
    - `cov_R_BI` (3×3 tangent-space, full)
    - `cov_R_BM` (3×3 tangent-space, full)
  - Nuisance snapshot (refined online):
    - `b_a`, `A_a`, `b_g`
    - optional `b_m`
    - current `R_m(t)` (3×3)
  - Quality / status:
    - `anchored` (bool)
    - `mag_reference_invalid` (bool)
    - `mag_disturbance_detected` (bool)
    - counts: `raw_samples`, `steady_segments`, `keyframes`
    - residual RMS: `accel_residual_rms`, `mag_residual_rms`
    - diversity: `gravity_max_angle_deg`, `mag_proj_max_angle_deg`

**Convenience transform stream**

- Topic: `ahrs_mount_transform` (`geometry_msgs/TransformStamped`)
- Publish: `T_BI` as a TF-style message for easy plotting/debugging
  - `header.frame_id = base_link`
  - `child_frame_id = imu_frame`
  - `transform.rotation` = `R_BI`
  - `transform.translation` SHALL be `[0,0,0]` exactly (constant)

#### 4.3.3 Diagnostics

- Topic: `ahrs_mount_diagnostics` (`diagnostic_msgs/DiagnosticArray`)
- Publish rate: 1–5 Hz (and on state transitions)

Report the following keys (at minimum):

- Timing / drops:
  - `imu_raw_dropped_no_calibration` (count)
  - `mag_dropped_bad_cov` (count)
  - `time_sync_slop_max_ns` (if tracked)
- Bootstrap / state:
  - `bootstrap_active` (bool)
  - `bootstrap_remaining_sec` (float)
  - `anchored` (bool)
- Steady/keyframe pipeline:
  - `steady_detected` (bool)
  - `steady_segments` (count)
  - `keyframes` (count)
  - `need_more_tilt` (bool)
  - `need_more_yaw` (bool)
- Magnetometer health:
  - `mag_reference_invalid` (bool)
  - `mag_disturbance_detected` (bool)
  - `R_m_trace` (float)
  - `mag_factors_dropped` (count)
- Solver health:
  - `optimizer_iterations_last` (int)
  - `optimizer_step_norm_last` (float)
  - `accel_residual_rms` (float)
  - `mag_residual_rms` (float)
- Persistence:
  - `last_save_unix_ns` (int)
  - `save_period_sec` (float)
  - `save_fail_count` (count)

### 4.4 Calibration lifecycle

The node **starts calibrating immediately** on startup and **continuously refines** the mount estimate online.

#### 4.4.1 Startup behavior (assume flat + stationary)

Constraint: at process start, `base_link` is **flat and stationary**.

On startup, the node:

- Begins stationary segmentation immediately.
- Enters an initial **bootstrap** phase for `bootstrap_sec` to characterize noise and lock in priors:
  - latch the **first valid** `imu_calibration` as the one-shot prior (per §2.3)
  - estimate gyro stationary statistics (mean + covariance / PSD) from `ω_corr`
  - estimate accel stationary statistics from `a_corr` (for gating and weights)
- Automatically captures the **reference pose** at the end of bootstrap (equivalent to §6.4 anchor):
  - set `R_WB,ref := I`
  - record reference gravity direction (from bootstrap stationary mean)
  - record reference magnetic direction if mag is valid (else mark `mag_reference_invalid = true`)

Defaults:

- `bootstrap_sec = 5.0`

Log when bootstrap is complete.

#### 4.4.2 Continuous calibration + periodic persist

The node runs the optimizer continuously as new stationary segments arrive and updates `T_BI`, `T_BM` whenever the solution changes.

Persistence policy:

- Write the current calibration to disk **every `save_period_sec`**, regardless of delta.
- Default: `save_period_sec = 2.0`

Notes:

- High write counts are acceptable.
- If reference capture occurred without valid mag, calibration is still saved but marked as gravity-only / `mag_reference_invalid = true` in metadata.

### 4.5 Parameters

This node is configured entirely via parameters using constants near the top of the file, with a one-line comment of descriptive documentation for each.

Numeric values shown as `TBD` are intended to be tuned later; the parameter names and semantics are the stable contract. `TBD` parameters may come from ROS, for example.

#### 4.5.1 Topics and frames

- `topics.imu_raw` (string): IMU raw topic name
  - default: `imu_raw`
- `topics.imu_calibration` (string): IMU calibration prior topic name
  - default: `imu_calibration`
- `topics.magnetic_field` (string): magnetometer topic name
  - default: `magnetic_field`

- `frames.base_frame` (string): body/base frame
  - default: `base_link`
- `frames.world_frame` (string): internal world/anchor frame name (internal use)
  - default: `world`

#### 4.5.2 Bootstrap and anchoring

- `bootstrap.bootstrap_sec` (double): duration of startup bootstrap window (flat + stationary assumption)
  - default: `5.0`
- `bootstrap.require_flat_stationary` (bool): enforce “flat + stationary at boot” assumption (else only best-effort)
  - default: `true`
- `bootstrap.mag_reference_required` (bool): if true, refuse to declare “fully anchored” without valid mag reference
  - default: `false`

#### 4.5.3 Steady detection (windowed gating)

- `steady.steady_sec` (double): minimum continuous steady duration before emitting a steady segment
  - default: `2.0`
- `steady.window_type` (string): windowing policy (e.g., `sliding`, `tumbling`)
  - default: `sliding`

Thresholds (tuned later; used in §5.1):

- `steady.omega_mean_thresh` (double): threshold on `||mean(ω_corr)||`
  - default: `TBD`
- `steady.omega_cov_thresh` (double): threshold on `trace(cov(ω_corr))`
  - default: `TBD` (often derived from bootstrap stats via `steady.k_omega`)
- `steady.a_cov_thresh` (double): threshold on `trace(cov(a_corr))`
  - default: `TBD`
- `steady.a_norm_min` (double): minimum plausible `||mean(a_corr)||` (m/s²)
  - default: `TBD`
- `steady.a_norm_max` (double): maximum plausible `||mean(a_corr)||` (m/s²)
  - default: `TBD`

Derived-threshold helpers:

- `steady.k_omega` (double): scale factor applied to bootstrap gyro covariance statistic to form `omega_cov_thresh`
  - default: `TBD`

#### 4.5.4 Keyframing / clustering

- `cluster.cluster_g_deg` (double): gravity-direction angular threshold for keyframe assignment (deg)
  - default: `7.5`
- `cluster.cluster_h_deg` (double): mag-direction angular threshold for keyframe assignment when mag is valid (deg)
  - default: `12.5`
- `cluster.K_max` (int): maximum number of active keyframes (0 = unlimited)
  - default: `0`
- `cluster.drop_policy` (string): policy when `K_max` exceeded (`redundant`, `oldest`, `lowest_information`)
  - default: `lowest_information`

#### 4.5.5 Motion diversity requirements

- `diversity.N_min` (int): minimum number of steady segments (or keyframes) before reporting “sufficient data”
  - default: `TBD`
- `diversity.tilt_min_deg` (double): minimum gravity-direction spread to be considered diverse enough (deg)
  - default: `TBD`
- `diversity.yaw_min_deg` (double): minimum mag-projection spread to be considered diverse enough (deg)
  - default: `TBD`
- `diversity.use_mag_for_yaw` (bool): if false, do not require yaw diversity even when mag is present
  - default: `true`

#### 4.5.6 Magnetometer direction-noise model (direction residual)

Conversion / gating for driver covariance (`Σ_m_raw`, tesla² → direction covariance, unitless²):

- `mag.s_min_T` (double): minimum `||m_raw||` allowed for covariance conversion (T)
  - default: `1e-6`
- `mag.use_driver_cov_as_prior` (bool): if true, attempt to initialize direction covariance from `Σ_m_raw` via Jacobian normalization; otherwise ignore `Σ_m_raw` entirely
  - default: `true`

Adaptive direction-noise model `R_m(t)` (unitless² ≈ rad²):

- `mag.Rm_init_diag` (double[3]): fallback diagonal initialization for `R_m0` when `Σ_dir` is unavailable
  - default: `[TBD, TBD, TBD]`
- `mag.Rm_min_diag` (double[3]): lower bound on `R_m(t)` (SPD clamp)
  - default: `[TBD, TBD, TBD]`
- `mag.Rm_max_diag` (double[3]): upper bound on `R_m(t)` (SPD clamp)
  - default: `[TBD, TBD, TBD]`
- `mag.Rm_alpha` (double): covariance-matching update rate
  - default: `TBD`
- `mag.disturbance_gate_enabled` (bool): enable mag disturbance detection / factor dropping
  - default: `true`
- `mag.disturbance_gate_sigma` (double): gating threshold for declaring mag disturbance (e.g., NIS or robust score)
  - default: `TBD`

#### 4.5.7 TF publishing and stability

- `tf.publish_dynamic` (bool): publish changing estimates to `/tf`
  - default: `true`
- `tf.publish_static_when_stable` (bool): publish to `/tf_static` once stable
  - default: `true`
- `tf.republish_static_on_save` (bool): republish `/tf_static` whenever the calibration file is updated
  - default: `true`

Stability detection (used by §4.3.1 “stable enough”):

- `stability.stable_window_sec` (double): window over which stability is evaluated
  - default: `TBD`
- `stability.stable_rot_thresh_rad` (double): max allowed rotation change magnitude `||Δθ||` over the stable window
  - default: `TBD`
- `stability.require_need_more_tilt_false` (bool): require `need_more_tilt == false` before declaring stable
  - default: `true`
- `stability.require_need_more_yaw_false` (bool): require `need_more_yaw == false` (when mag is used) before declaring stable
  - default: `true`

#### 4.5.8 Persistence / output file

- `save.save_period_sec` (double): period for writing the calibration snapshot to disk
  - default: `2.0`
- `save.output_path` (string): output YAML/JSON file path
  - default: `TBD` (e.g., a package/config path or runtime-writable directory)
- `save.format` (string): file format (`yaml` or `json`)
  - default: `yaml`
- `save.atomic_write` (bool): write via temp file + atomic rename
  - default: `true`

#### 4.5.9 Optimizer settings (online SO(3) solve)

- `solver.backend` (string): solver backend identifier (`gn`, `lm`, etc.)
  - default: `TBD`
- `solver.max_iters` (int): max iterations per update
  - default: `TBD`
- `solver.update_rate_hz` (double): maximum optimizer update rate
  - default: `TBD`
- `solver.robust_loss` (string): robust loss type (`huber`, `cauchy`, etc.)
  - default: `TBD`
- `solver.robust_scale_a` (double): robust scale for accel direction residuals
  - default: `TBD`
- `solver.robust_scale_m` (double): robust scale for mag direction residuals
  - default: `TBD`

Regularization (prevents “mount absorbs bias”):

- `solver.reg_mount_vs_bias` (double): strength of regularization discouraging mount rotation from compensating nuisance biases
  - default: `TBD`

#### 4.5.10 Diagnostics

- `diag.publish_rate_hz` (double): diagnostics publish rate
  - default: `TBD`
- `diag.track_time_sync_slop` (bool): track and report time-sync slop metrics
  - default: `true`

---

## 5. Motion calibration procedure

The node estimates mount rotations by collecting **multiple stationary segments** spanning different attitudes (tilts and yaw changes). Full inversion is not required.

### 5.1 Quasi-stationary (“steady”) detection

We want to accept “held mostly steady” poses (small hand motion) and only create a new sample point when the device has been steady for long enough.

Define a sliding window of length `steady_sec` (default 2.0 s). Let gyro be `ω_corr(t)` and accel be `a_corr(t)`.

A window is **STEADY** if all hold:

- Mean gyro magnitude is small:
  - `||mean(ω_corr)|| < ω_mean_thresh`
- Gyro variation is small:
  - `trace(cov(ω_corr)) < ω_cov_thresh`
- Accel variation is small:
  - `trace(cov(a_corr)) < a_cov_thresh`
- Accel magnitude is plausible (reject gross motion):
  - `a_norm_min < ||mean(a_corr)|| < a_norm_max`

When the stream transitions from NOT_STEADY → STEADY and remains STEADY continuously for `steady_sec`,
emit one **steady segment** `S_k` with:

- mean accel `\bar a_I,k`, covariance `Σ_a,k`
- mean mag `\bar m_M,k`, covariance `Σ_m,k` (if mag available; else mark missing)
- duration and timestamps

Notes:

- This is intentionally looser than strict “stationary.” It tolerates small hand jitter as long as the statistics stay tight.
- Use bootstrap-estimated gyro stats to initialize thresholds:
  - `ω_cov_thresh` derived from stationary `cov(ω_corr)` during bootstrap (scaled by factor `k_ω`, default 3–5×).

### 5.2 Motion diversity requirement

To solve rotations robustly:

- require at least `N_min` stationary segments (e.g., 10)
- require tilt diversity (e.g., max angle between gravity directions across segments > 30°)
- require yaw diversity (mag projection diversity) if possible

If diversity is insufficient, keep collecting and publish a diagnostic like:

- `need_more_tilt = true`
- `need_more_yaw = true`

### 5.3 Segment clustering + keyframes (pose diversity)

Held poses can repeat (or be too similar). We cluster steady segments into **keyframes** so the optimizer
gets diverse constraints and we avoid overweighting one attitude.

Define per-segment unit directions:

- `g_k := normalize(-\bar a_I,k)` (gravity “up” direction in `{I}`)
- `h_k := normalize( \bar m_M,k )` (mag direction in `{M}`), if available

Maintain a set of keyframes `K_j`. Each keyframe stores running means/covariances for `(g, h)` and counts.

Assignment rule (online):

- For a new segment `S_k`, compute similarity to each keyframe:
  - `Δg = angle(g_k, g_j)`
  - if mag valid: `Δh = angle(h_k, h_j)` else ignore mag
- Assign to the closest keyframe if:
  - `Δg < cluster_g_deg` AND (if mag valid) `Δh < cluster_h_deg`
- Otherwise create a new keyframe.

Defaults:

- `cluster_g_deg = 7.5`
- `cluster_h_deg = 12.5`

Optimization dataset policy:

- The optimizer runs on the current set of keyframes (one “representative” constraint per cluster),
  using the keyframe’s aggregated mean/covariance as the measurement.
- Optionally cap keyframes at `K_max` by dropping the lowest-information ones (e.g., most redundant gravity directions).

This provides pose diversity automatically while allowing the user to “hold kinda steady” for ~2 s per pose.

---

## 6. Mathematical model

This section defines the estimation problem for continuous mounting calibration. Measurements are accumulated into **steady segments** and then **clustered into keyframes** (§5.3). The optimizer runs over keyframes.

### 6.1 Unknowns to estimate (outputs + nuisance)

Primary outputs:

- `R_BI` : rotation IMU→body
- `R_BM` : rotation mag→body

**No translations:**

- `p_BI` and `p_BM` are fixed constants under the co-location contract:
  - `p_BI := [0,0,0]`
  - `p_BM := [0,0,0]`
- The optimizer SHALL NOT include any translation variables.

Nuisance (internal, continuously refined):

- IMU systematic parameters (live in state):
  - accel: `a_corr = A_a (a_raw − b_a)` where `b_a, A_a` are estimated
  - gyro: `ω_corr = ω_raw − b_g` where `b_g` is estimated
- Magnetometer offset: `m_corr = m_raw − b_m`
- World directions (unit vectors):
  - `g_W` gravity direction in `{W}`
  - `m_W` magnetic field direction in `{W}`
- Per-keyframe body attitude:
  - `R_WB,j` for each keyframe `j`
- Measurement noise models (refined online):
  - IMU steady-window statistics (used for weighting / gating)
  - `R_m(t)` magnetometer noise covariance (adaptive, bounded SPD, §2.4)

**Prior semantics:**

- `imu_calibration` and any mag-cal inputs are **priors only**; they initialize the nuisance parameters and their covariances.
- During operation, nuisance parameters are refined using accumulated steady/keyframe data (robust, continuous estimation), not “locked”.

---

### 6.2 Keyframe measurement factors (direction constraints)

For keyframe `j`, we form direction observations from aggregated steady data:

- IMU “up” direction in `{I}`:
  - `â_I,j = normalize( -\bar a_I,j )`
- Magnetometer direction in `{M}` (if valid):
  - `m̂_M,j = normalize( \bar m_M,j )`

Predicted directions (same frame as measurement):

Rotation chains:

- `R_WI,j = R_BIᵀ * R_WB,j`
- `R_WM,j = R_BMᵀ * R_WB,j`

So:

- `R_IW,j = R_WB,jᵀ * R_BI`
- `R_MW,j = R_WB,jᵀ * R_BM`

Predictions:

- `â̂_I,j = (R_WB,jᵀ * R_BI) * g_W`
- `m̂̂_M,j = (R_WB,jᵀ * R_BM) * m_W`

Residuals (direction-only, robust to magnitude):

- accel: `r_a,j = â_I,j × â̂_I,j`
- mag: `r_m,j = m̂_M,j × m̂̂_M,j`

Weighting:

- `w_a,j` from the steady-window / keyframe aggregated accel covariance (and duration / sample count)
- `w_m,j` from adaptive `R_m(t)` and keyframe aggregated mag covariance
- If mag is flagged disturbed for the keyframe, drop the mag factor for that keyframe (gravity-only constraint).

---

### 6.3 Joint optimization structure (online)

Solve for:

- global: `R_BI`, `R_BM`, `g_W`, `m_W` (and nuisance IMU/mag params)
- per-keyframe: `R_WB,j`

Objective:
\[
\min \sum_j \rho_a(\|r_{a,j}\|^2, w_{a,j}) + \rho_m(\|r_{m,j}\|^2, w_{m,j})
+ \text{priors}
\]

Priors:

- Unit constraints: `||g_W|| = 1`, `||m_W|| = 1`
- Rotations live on SO(3): `R_BI, R_BM, R_WB,j ∈ SO(3)`
- Systematic priors:
  - initialize `(b_a, A_a, b_g)` from the first valid `imu_calibration`
  - initialize R_m0 from Σ_dir (computed from Σ_m_raw via normalization Jacobian, §2.4); else use defaults
  - initialize `b_m` from a provided mag offset prior, if available
- Weak regularization on nuisance parameters to prevent “mount absorbs bias”.

**No translation terms:**

- The objective SHALL NOT include translation priors, translation regularization, or baseline constraints because translations are fixed by the co-location contract.

---

### 6.4 Reference anchoring (ties result to `base_link`)

There is a global rotational gauge freedom unless anchored.

In this system, anchoring is automatic (per §4.4):

- At end of bootstrap, assume `base_link` is flat + stationary and set:
  - `R_WB,ref := I`
- The world frame `{W}` is defined as “body at the reference pose”.
- All `R_WB,j` are solved relative to this anchor.

This makes `R_BI` and `R_BM` directly interpretable as IMU→body and mag→body.

---

### 6.5 Initialization (deterministic)

1. Bootstrap (`bootstrap_sec`): collect steady data, estimate gyro/accel stationary statistics, latch priors.
2. Set `R_WB,ref = I` and initialize:
   - `g_W := [0, 0, -1]` (or derived from reference accel)
   - `m_W := normalize(reference mag)` if mag valid
3. Initialize `R_BI, R_BM := I` (or from URDF if available).
4. For each new keyframe `j`, seed `R_WB,j` using TRIAD-like construction (gravity + mag when available).
5. Run iterative optimization on SO(3) (GN/LM) continuously as keyframes update.

### 6.6 Calibration algorithm (summary)

Pipeline:

1. Bootstrap (startup, flat + stationary): latch `imu_calibration` as priors, estimate steady/noise stats, set `R_WB,ref := I`, initialize `g_W` and `m_W`.
2. Continuous STEADY detection (`steady_sec`): emit steady segments `S_k` from windowed statistics on `ω_corr, a_corr`.
3. Cluster segments into keyframes `K_j` using angular thresholds on gravity (and mag if valid).
4. Optimize online over `{R_BI, R_BM, g_W, m_W, R_WB,j}` using direction residuals (`×`) with robust loss; update adaptive mag noise `R_m(t)`.
5. Publish TF and persist snapshot every `save_period_sec`.

---

## 7. Outputs and persistence format

### 7.1 What gets saved

Save a single self-contained calibration snapshot including including **mount rotations** and **nuisance parameters** used by the estimator.

**Mount rotations (primary outputs)**

- `R_BI` and `R_BM`:
  - rotation quaternion (explicit convention, e.g. `quaternion_wxyz`)
  - rotation covariance 3×3 in tangent space (full matrix; do not diagonalize)

Translations are fixed constants under the co-location contract:

- `p_BI = p_BM = [0,0,0]` meters.
- Translations are not estimated and are not persisted

**IMU systematic parameters (nuisance, refined online)**

- Accel correction parameters:
  - `b_a` (m/s²), 3×1
  - `A_a` (3×3, row-major)
  - `cov(b_a, A_a)` (12×12, row-major) if maintained
- Gyro bias:
  - `b_g` (rad/s), 3×1
  - `cov(b_g)` (3×3, row-major)

**Magnetometer nuisance (refined online)**

- Optional mag offset:
  - `b_m` (tesla), 3×1
  - `cov(b_m)` (3×3) if maintained
- Adaptive mag measurement noise model:
  - `R_m` (3×3, unitless² ≈ rad²), current bounded SPD value for the direction residual `r_m`

**Estimator bookkeeping / quality metadata**

- Data volume:
  - counts: raw samples ingested, steady segments accepted, keyframes active
  - total duration used
- Conditioning / diversity:
  - gravity max-angle (deg)
  - mag projection max-angle (deg), if mag valid
- Fit quality:
  - accel residual RMS (direction residual)
  - mag residual RMS (direction residual), if mag valid
  - robust inlier ratio (optional)
- Flags:
  - `anchored: true|false`
  - `mag_reference_invalid: true|false`
  - `mag_disturbance_detected: true|false` (if any keyframes dropped mag)

NOTE: `format_version` is held at `1` during development and will be bumped when the on-disk schema is production-frozen.

### 7.2 File format (YAML)

`ahrs_mount.yaml`:

```yaml
format_version: 1

frames:
  base_frame: base_link
  imu_frame: imu_link
  mag_frame: mag_link

flags:
  anchored: true
  mag_reference_invalid: false
  mag_disturbance_detected: false
  mag_dir_prior_from_driver_cov: false

mounts:
  R_BI:
    quaternion_wxyz: [1.0, 0.0, 0.0, 0.0]
    rot_cov_rad2: [[...], [...], [...]]
  R_BM:
    quaternion_wxyz: [1.0, 0.0, 0.0, 0.0]
    rot_cov_rad2: [[...], [...], [...]]

nuisance:
  imu:
    accel_bias_mps2: [0.0, 0.0, 0.0] # b_a
    accel_A_row_major: [... 9 ...] # A_a
    accel_param_cov_row_major_12x12: [... 144 ...]
    gyro_bias_rads: [0.0, 0.0, 0.0] # b_g
    gyro_bias_cov_row_major_3x3: [... 9 ...]
  mag:
    offset_t: [0.0, 0.0, 0.0] # b_m
    offset_cov_row_major_3x3: [... 9 ...]
    R_m_unitless2_row_major_3x3: [... 9 ...]
    R_m0_unitless2_row_major_3x3: [... 9 ...]

quality:
  raw_samples: 0
  steady_segments: 0
  keyframes: 0
  total_duration_sec: 0.0
  accel_residual_rms: 0.0
  mag_residual_rms: 0.0
  diversity:
    gravity_max_angle_deg: 0.0
    mag_proj_max_angle_deg: 0.0
```
