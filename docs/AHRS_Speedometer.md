# AHRS_Speedometer.md

> **Goal:** Estimate **forward (longitudinal) speed** for Falcon/train from IMU signals, with **bounded drift** via **ZUPT (Zero-Velocity Updates)** detected from a trusted _tilt_ signal + motor command (duty cycle).
> **Non-goals (v1):** full 3D pose, global heading, map frame, absolute position, or full IMU mounting calibration.

---

## 1. Overview

This module provides a **1D inertial speedometer**:

- Estimates **forward speed** `v_f` (m/s) along **body x-forward**.
- Learns the **forward accelerometer bias component** `b_f` robustly during stops (ZUPT).
- Uses a trusted **tilt** signal for:
  - stationary detection (low angular rate + stable roll/pitch)
  - estimating the **gravity projection** onto the forward axis (to remove gravity from raw accel)
- Optionally uses **6DOF accel calibration** as _priors_ (not as ground truth) to initialize scale/bias uncertainty.

Two ROS nodes are defined:

1. `zupt_sensor_node`
   Detects stops; publishes a zero-velocity “measurement” with covariance.
2. `speedometer_node`
   Runs a 1D Kalman filter to estimate forward speed (and forward bias, and optionally gravity magnitude).

---

## 2. Frames and sign conventions

- **body frame:** `base_link`
  - `+x` = **forward** (longitudinal direction)
  - `+y` = left
  - `+z` = up
- **IMU raw frame:** `imu_raw` (conceptual; represented by `/imu_raw` message frame_id)
- **Tilt frame:** `/tilt` is published with `frame_id: imu_link` (derived estimate in IMU frame)

For v1, you may treat:

- `base_link ≈ imu_link` rotation (identity) if you only care about 1D speed and accept some scale/bias error.
- The _gravity compensation_ uses tilt’s roll/pitch (yaw irrelevant for gravity).

---

## 3. ROS Interfaces

### 3.1 Inputs

#### `/oasis/<system>/tilt` _(trusted)_

**Type:** `sensor_msgs/Imu`
Used for:

- `orientation` (roll/pitch only; yaw may be unobservable / high covariance)
- `angular_velocity` (used for stationary detection)
- `orientation_covariance`, `angular_velocity_covariance` (used for adaptive thresholds / confidence)

#### `/oasis/<system>/conductor_state`

**Type:** `oasis_msgs/msg/ConductorState`
Used for:

- `duty_cycle` in `[-1, 1]`
  Used as a discrete “intent” signal: `== 0` means commanded stop (controller deadzone already applied).

#### `/oasis/<system>/imu_raw`

**Type:** `sensor_msgs/Imu`
Used for:

- `linear_acceleration` (raw, biased, includes gravity)
- (optionally) `angular_velocity` if tilt angular velocity is unavailable

> **Note:** `imu_raw` acceleration magnitude may be wrong (e.g., ~10.17 m/s² at rest). This design does **not** require that `|a|≈9.81`.

#### `/oasis/<system>/imu_calibration` _(optional priors)_

**Type:** `oasis_msgs/msg/ImuCalibration`
Used as priors only:

- accel scale/misalignment matrix `A` and bias `b_a`
- parameter covariances (if provided)
- gyro bias prior (optional)

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
- thresholds (omega_enter/exit, tilt stability)
- v_f, P_vv
- b_f, P_bb
- gravity estimate (if enabled) + covariance
- innovation stats for ZUPT updates
- gating reasons (why ZUPT rejected)

---

## 4. ZUPT Sensor Node (`zupt_sensor_node`)

### 4.1 Stationary detection logic (tilt + duty)

Stationary should mean:

- robot is not commanded to move **and**
- robot is not rotating significantly **and**
- roll/pitch are stable (optional but helpful against vibration false positives)

**Inputs used:**

- `duty_cycle` from `/conductor_state`
- `ω = tilt.angular_velocity` (or imu_raw gyro)
- `q = tilt.orientation` (roll/pitch stability)

**Gating rules (typical):**

1. `duty_cycle == 0` (exact zero due to deadzoning)
2. Angular rate magnitude below threshold:
   - `||ω|| < omega_enter` to enter stationary
   - `||ω|| > omega_exit` to exit stationary (hysteresis)
3. Optional: tilt stability (roll/pitch change rate small)
   - e.g., `angle( q(t), q(t-Δt) ) < tilt_delta_max`

### 4.2 Adaptive threshold “annealing”

Because vibration and sensor conditions vary, thresholds can be adaptive:

- Maintain running estimates during recent motion/stops:
  - gyro noise floor `σ_ω` from covariance or from a short window variance estimate
  - optionally gyro bias estimate `b_g` (see §4.4)

Then set:

- `omega_enter = k_enter * σ_ω + omega_min`
- `omega_exit  = k_exit  * σ_ω + omega_min`, with `k_exit > k_enter`

### 4.3 ZUPT measurement covariance annealing (confidence grows with stillness)

When stationary persists for longer, the ZUPT velocity measurement can be trusted more:

- Let `t_s` = time continuously stationary.
- Define a decreasing function:
  - `σ_zupt_v(t_s) = max(σ_min, σ0 * exp(-t_s / τ))`

So the ZUPT “pull to zero” is gentle initially, then becomes strong if the robot stays still.

This avoids instantly “snapping” velocity to zero on a brief pause.

### 4.4 Gyro bias estimate (optional, but helps stability)

Even if speedometer is 1D, you may want a gyro bias estimate in ZUPT sensor to avoid false motion detection:

- During stationary, treat true angular rate as ~0:
  - measurement: `ω_meas = b_g + noise`
- Run a 3D bias estimator:
  - `b_g` as a slowly-updated mean with covariance
  - or a tiny Kalman filter per axis

This bias does **not** have to be perfect; it’s mainly to keep `||ω||` gating stable.

**Publish?**

- If useful elsewhere, publish `b_g` as part of diagnostics (or a custom msg).

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

At each IMU raw sample:

1. Get raw acceleration vector from `/imu_raw`:
   - `a_raw_I` (m/s²) in IMU raw frame

2. Compute forward axis unit vector `e_f` in the same frame:
   - v1: assume `e_f = [1,0,0]` in imu frame if `base_link≈imu_link`
   - if you later add mounting, compute `e_f` via `R_BI`

3. Remove gravity using tilt:
   - From `tilt.orientation` (roll/pitch), compute gravity direction in IMU frame:
     - `ĝ_I = R(q_tilt)ᵀ * [0,0,1]` (or `[0,0,-1]` depending on convention)
   - Use a gravity magnitude:
     - either fixed parameter `imu_gravity_mps2`
     - or learned `g` (state), or
     - or a prior from tilt (see §5.6)

4. Form forward specific force estimate:
   - `a_f = e_fᵀ * a_raw_I  -  g * (e_fᵀ * ĝ_I)`
     This estimates “horizontal/forward acceleration” excluding gravity projection.

> You are **not** depending on the accelerometer being well-calibrated; `b_f` absorbs a large part of the error along forward.

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

- Initialize `g` from:
  - a parameter or a prior derived from tilt history
- Then refine `g` slowly during ZUPT windows using the measured accel magnitude statistics.

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
- `tilt_delta_max_rad` (optional)
- `min_stationary_sec` (debounce, e.g. `0.2`)
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
- Tilt angular velocity is reasonably clean for stationary gating

### Drift sources:

- forward accel bias changes with temperature/vibration
- scale error (if not modeled)
- imperfect gravity projection from tilt under vibration
- coasting / creep when duty=0 (train not truly stationary)

### What you cannot get from yaw-only motion:

- Full IMU mounting (`R_BI`) is not observable without tilt diversity.
- But for 1D speed, you can proceed with approximate alignment and learn bias.

---

## 9. Testing plan

### Offline bag tests

- Log `/tilt`, `/imu_raw`, `/conductor_state`, `/zupt`, `/forward_twist`
- Check:
  - ZUPT triggers only when expected
  - velocity returns to ~0 during stops
  - covariance grows during motion and shrinks during long stops

### Bench tests

- Hold train still, duty=0:
  - verify ZUPT assert after debounce
  - verify velocity covariance anneals down

### Track tests

- Run laps with periodic stops:
  - verify drift between stops is bounded
  - verify speed sign matches direction

---

## 10. Future extensions

- Add `s_f` scale factor learning with priors from `imu_calibration.accel_a`
- Add coasting detector (if duty=0 but vibration signature indicates motion)
- Add wheel encoder / hall sensor for absolute speed correction
- Add mounting calibration later (AHRS Mounting) to unify frames robustly
- Publish `nav_msgs/Odometry` if you later want a standard interface (twist only)

---
