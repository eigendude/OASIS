# AHRS_Speedometer.md

> **Goal:** Estimate **forward (longitudinal) speed** for Falcon/train from IMU signals, with **bounded drift** via **ZUPT (Zero-Velocity Updates)** detected from **raw gyro quietness** + motor command (duty cycle).
> **Non-goals (v1):** full 3D pose, global heading, map frame, absolute position, or full IMU mounting calibration.

---

## 1. Overview

This module provides a **1D inertial speedometer**:

- Estimates **forward speed** `v_f` (m/s) along **body x-forward**.
- Learns the **forward accelerometer bias component** `b_f` robustly during stops (ZUPT).
- Uses **raw IMU gyro** for stationary detection (quiet angular rate + duty-cycle gating).
- Treats gravity projection onto the forward axis as negligible on level track (v1); any residual coupling is absorbed into the learned bias during frequent ZUPTs.
- Optionally uses **6DOF accel/gyro calibration** as _priors_ (not as ground truth) to initialize bias uncertainty.

Two ROS nodes are defined:

1. `zupt_detector_node`
   Detects stops; publishes a zero-velocity “measurement” with covariance.
2. `speedometer_node`
   Runs a 1D Kalman filter to estimate forward speed (and forward bias).

---

## 2. Frames and sign conventions

- **body frame:** `base_link`
  - `+x` = **forward** (longitudinal direction)
  - `+y` = left
  - `+z` = up
- **IMU frame:** `imu_link` (from `/imu_raw.header.frame_id`)

For v1, you may treat:

- `base_link ≈ imu_link` rotation (identity) if you only care about 1D speed and accept some alignment error.
- The speedometer integrates acceleration along the assumed forward axis; ZUPT bounds drift and learns the effective forward bias.

---

## 3. ROS Interfaces

### 3.1 Inputs

#### `/oasis/<system>/conductor_state`

**Type:** `oasis_msgs/msg/ConductorState`
Used for:

- `duty_cycle` in `[-1, 1]`
  Used as a discrete “intent” signal: `== 0` means commanded stop (controller deadzone already applied).

#### `/oasis/<system>/imu_raw`

**Type:** `sensor_msgs/Imu`
Used for:

- `linear_acceleration` (raw, biased, includes gravity)
- `angular_velocity` (used for stationary detection)
- `angular_velocity_covariance` (used for gating / confidence)

> **Note:** `imu_raw` acceleration magnitude may be wrong (e.g., not exactly 9.81 m/s² at rest). This design does **not** require `|a|≈9.81`.

#### `/oasis/<system>/imu_calibration` _(optional priors)_

**Type:** `oasis_msgs/msg/ImuCalibration`
Used as priors only:

- accel bias prior (optional) and related covariance if provided
- gyro bias prior and covariance (recommended for ZUPT gating stability)

> **QoS note:** `imu_raw` should use SensorData QoS. `conductor_state` should be Reliable + Transient Local (late-joiners get the most recent duty state).

---

### 3.2 Outputs

#### `/oasis/<system>/zupt`

**Type:** `geometry_msgs/TwistWithCovarianceStamped`
Represents a **measurement** that forward velocity is zero:

- `twist.twist.linear.x = 0.0`
- `twist.covariance[0,0] = σ²_zupt_v` (tunable / annealed)
- All other covariance entries may be large or left as-is.

`header.frame_id`: `base_link` (preferred) or `imu_link` if base_link is not meaningful yet.

#### `/oasis/<system>/zupt_flag`

**Type:** `std_msgs/Bool`
Convenience boolean: `true` while stationary is asserted.

#### `/oasis/<system>/forward_twist`

**Type:** `geometry_msgs/TwistWithCovarianceStamped`
Speedometer output:

- `twist.twist.linear.x = v_f`
- `twist.covariance[0,0] = P_vv`

Frame: `base_link`.

#### `/oasis/<system>/speedometer_diagnostics` _(recommended)_

**Type:** `diagnostic_msgs/DiagnosticArray` (or a custom msg)

Suggested fields:

- stationary flag, dwell time
- thresholds (gate_enter/exit, duty gating, debounce timers)
- v_f, P_vv
- b_f, P_bb
- (optional) gravity estimate (if enabled) + covariance
- innovation stats for ZUPT updates
- gating reasons (why ZUPT rejected)

---

## 4. ZUPT Detector Node (`zupt_detector_node`)

### 4.1 Stationary detection logic (imu_raw gyro + duty)

Stationary should mean:

- robot is not commanded to move **and**
- robot is not rotating significantly

**Inputs used:**

- `duty_cycle` from `/conductor_state`
- `ω = imu_raw.angular_velocity`
- optionally: gyro bias prior from `/imu_calibration`

**Gating rules (typical):**

1. `duty_cycle == 0` (exact zero due to deadzoning; optionally allow epsilon)
2. Angular rate quietness gate (with hysteresis):
   - enter stationary when `d2(ω - b_g) < gate_enter`
   - exit stationary when `d2(ω - b_g) > gate_exit`
   where `d2(·)` is a Mahalanobis distance using `angular_velocity_covariance + gyro_bias_cov`.
3. Debounce (time-in-state):
   - require `min_stationary_sec` quiet time to enter
   - require `min_exit_sec` loud time to exit

### 4.2 Adaptive threshold scaling (optional)

If sensor conditions vary, thresholds can adapt:

- Maintain an EWMA of recent `d2` while duty is zero
- Scale `gate_enter/gate_exit` within bounds `[scale_min, scale_max]`

This helps avoid false positives from vibration changes without hardcoding a single gate.

### 4.3 ZUPT measurement covariance annealing (confidence grows with stillness)

When stationary persists longer, the ZUPT velocity measurement can be trusted more:

- Let `t_s` = time continuously stationary.
- Use a decreasing function:
  - `σ_zupt_v(t_s) = max(σ_min, σ0 * exp(-t_s / τ))`

So ZUPT “pull to zero” is gentle initially, then becomes strong during a long stop.

### 4.4 Gyro bias handling

ZUPT gating is much more stable if you account for gyro bias:

- Prefer a prior bias + covariance from `/imu_calibration` when available.
- If calibration is unavailable, treat bias as zero and rely on covariance + conservative gates.
- (Optional future) estimate a slow bias online only during stationary windows.

The node publishes:

- `/zupt_flag` boolean
- `/zupt` zero-velocity measurement with annealed variance

---

## 5. Speedometer Node (`speedometer_node`)

### 5.1 State, units, and covariance

A minimal 1D Kalman filter:

**State**

- `x = [ v_f, b_f ]ᵀ`
  - `v_f` = forward speed (m/s)
  - `b_f` = forward accel bias component (m/s²)

**Covariance**

- `P = cov(x)` (2×2)

**Optional extension**

- Add gravity magnitude `g` as a third state:
  - `x = [ v_f, b_f, g ]ᵀ`
  - Useful if `|a|` at rest is not trustworthy and you want the filter to learn a consistent gravity magnitude during ZUPT.

### 5.2 Inputs used by the propagation step

At each `/imu_raw` sample:

1. Get raw acceleration vector from `/imu_raw`:
   - `a_raw_I` (m/s²) in IMU frame.

2. Compute the forward axis unit vector `e_f` in the same frame:
   - v1: assume `e_f = [1,0,0]` in IMU frame if `base_link≈imu_link`
   - later: if you add mounting, compute `e_f` via `R_BI`.

3. Form forward acceleration used for integration:
   - `a_f = e_fᵀ * a_raw_I`

**Gravity note (v1):**

- Full IMU mounting (`R_BI`) is not observable without richer excitation / independent references (e.g., varied orientations, external heading/velocity cues).
- On level track, gravity projects mostly into IMU `z`, so the forward (`x`) projection is small. But note that track may not be level.
- Any residual gravity coupling into the forward axis (e.g., slight pitch) is treated as part of the learned forward bias `b_f`, and is bounded by frequent ZUPTs.
- If the vehicle runs sustained grades/pitch changes between ZUPTs, gravity leakage into `x` will appear as an acceleration bias and can induce speed drift between stops.

### 5.3 Process model (prediction)

Discrete time step `Δt`:

- `v_{k+1} = v_k + (a_f - b_f) * Δt`
- `b_{k+1} = b_k` (random walk)

Process noise:

- `q_v` from accel noise / model mismatch
- `q_b` sets how fast bias is allowed to wander (tunable)

This can be implemented as standard linear Kalman predict with:

- `F = [[1, -Δt], [0, 1]]`
- `u = a_f * Δt`
- `Q` based on `q_v`, `q_b`

### 5.4 ZUPT measurement update

When `/zupt` arrives (or when `zupt_flag==true` and you choose to update every N ms):

Measurement:

- `z = 0` (forward velocity is zero)

Model:

- `z = H x + noise`, with `H = [1, 0]`

Innovation covariance:

- `S = HPHᵀ + R_zupt`, where `R_zupt = σ²_zupt_v`

Update pulls:

- `v_f → 0`
- and indirectly learns `b_f` (because persistent ZUPTs constrain drift, causing bias to be corrected).

### 5.5 Learning forward accel bias robustly

**Key claim:** ZUPT lets you learn **only what’s observable** in 1D:

- The forward component of accelerometer bias (and scale, if you model it) is the most robust learnable term.
- Cross-axis calibration and full 3D bias are not observable without richer motion/tilt excitation.

If you want a **scale factor** too, extend the model:

- Add `s_f` (unitless) such that:
  - `v_{k+1} = v_k + (s_f * a_f - b_f)Δt`
- But be careful: `s_f` and `b_f` can be weakly identifiable without varied acceleration profiles. Use strong priors.

### 5.6 Using `imu_calibration` as priors (optional)

You want priors (that’s the point of your 6DOF calibration), but you **don’t** want to trust them blindly.

Recommended pattern:

- Use `imu_calibration` to initialize:
  - `b_f` prior mean and covariance (weak prior)
  - optionally a scale prior `s_f` if you add it
  - optionally gyro bias prior for ZUPT gating

But keep priors **wide**:

- If the calibration is stale/wrong, the ZUPT-driven updates should override it quickly.

If you include gravity magnitude `g` as a state:

- Initialize `g` from a parameter (default ~9.80665), with a wide prior.
- Optionally refine `g` slowly during long ZUPT windows using statistics of `|a_raw|` (but only if your sensor/model makes that meaningful).

---

## 6. Why magnetometer is not required for speedometer v1

For **forward speed only**, magnetometer is usually unnecessary:

- Yaw is not needed if you define the forward axis in the body/IMU frame and only integrate along that axis.
- Magnetometer is more relevant for:
  - heading / yaw drift correction
  - mapping accel into a world frame
  - coupling speed to global velocity

You can add mag later if you decide to estimate world-frame velocity or fuse odometry.

---

## 7. Parameters

Suggested ROS parameters (names are illustrative):

### ZUPT sensor

- `duty_zero_epsilon` (default: `0.0` if deadzoning guarantees exact zero)
- `omega_enter` / `omega_exit` (or adaptive gains `k_enter`, `k_exit`)
- `min_stationary_sec` (debounce, e.g. `0.2`)
- `min_exit_sec` (debounce, e.g. `0.05`)
- `use_mahalanobis_gate` (bool, optional)
- `gate_enter_chi2` / `gate_exit_chi2` (if using Mahalanobis)
- `omega_norm_enter` / `omega_norm_exit` (if using norm gate)
- `zupt_sigma_v0` (initial σ)
- `zupt_sigma_v_min`
- `zupt_sigma_tau_sec` (annealing time constant)

### Speedometer

- `process_noise_v` (q_v)
- `process_noise_b` (q_b)
- `initial_speed_sigma`
- `initial_bias_sigma`
- `imu_gravity_mps2` (if fixed)
- `use_gravity_state` (bool)
- `publish_rate_hz`
- `frame_id` (`base_link`)

---

## 8. Expected behavior and limitations

### Works well when:

- Train does frequent stops (station platforms / pauses / manual stops)
- Duty command provides a reliable “intent” signal
- Raw gyro is reasonably clean for stationary gating

### Drift sources:

- forward accel bias changes with temperature/vibration
- gravity leakage into forward axis due to pitch/grade (v1 treats this as bias; frequent ZUPTs bound it)
- imperfect axis alignment between `base_link` and `imu_link`
- coasting / creep when duty=0 (train not truly stationary)

### What you cannot get from yaw-only motion:

- Full IMU mounting (`R_BI`) is not observable without tilt diversity.
- But for 1D speed, you can proceed with approximate alignment and learn bias.

---

## 9. Future extensions

- Add `s_f` scale factor learning with priors from `imu_calibration.accel_a`
- Add coasting detector (if duty=0 but vibration signature indicates motion)
- Add wheel encoder / hall sensor for absolute speed correction
- Add mounting calibration later (AHRS Mounting) to unify frames robustly
- Publish `nav_msgs/Odometry` if you later want a standard interface (twist only)

---
