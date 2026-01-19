# AHRS Structure and Implementation Spec

This document defines a **from-scratch AHRS rewrite** with a clean folder
structure, class-based design, and explicit math. It is written so each
component can be implemented independently without relying on unstated
Kalman filter knowledge.

---

## 1. Goals and Non-Goals

### Goals
- A clear **core vs. ROS integration** split
- A small set of **reusable, ROS-agnostic classes** in `src/`
- **Explicit math** for all estimation steps (no hidden assumptions)
- Predictable, testable class interfaces

### Non-Goals
- No global navigation or map alignment
- No absolute position estimation
- No advanced calibration (hard/soft-iron, temperature) beyond what is
  specified here

---

## 2. Proposed Directory Layout

Create a new directory tree under the AHRS package with grouped
subdirectories and class-based modules.

```
oasis_control/oasis_control/localization/ahrs/
  __init__.py
  src/
    core/
      math/
        quat.py
        linalg.py
      time/
        clock.py
        timeline.py
      models/
        imu_model.py
        mag_model.py
        process_model.py
      state/
        ahrs_state.py
        ahrs_error_state.py
        ahrs_params.py
      filters/
        eskf.py
      bootstrap/
        initial_alignment.py
      io/
        conversions.py
        types.py
    ros/
      msgs.py
      conversions.py
      publishers.py
      ros_clock.py
  nodes/
    ahrs_node.py
```

**Notes**
- All estimation math and state live in `src/core/...` with **no ROS
  imports**
- `src/ros/...` contains **ROS message conversion only**
- `nodes/` contains the ROS node wrapper

---

## 3. Common Conventions

### 3.1 Frames
- `{W}` world frame (ROS `world`), ENU: +X east, +Y north, +Z up
- `{B}` body frame (ROS `base_link`): +X forward, +Y left, +Z up
- `{I}` IMU frame (from `imu_raw.header.frame_id`)
- `{M}` magnetometer frame (from `magnetic_field.header.frame_id`)

### 3.2 Rotations and Quaternions
- `q_WB` rotates a vector **from world to body**
- Rotation matrix `R_WB` is derived from `q_WB`
- Quaternion multiplication `q1 ⊗ q2` applies `q2` then `q1`
- Convert body vector to world: `v_W = R_WBᵀ v_B`
- Convert world vector to body: `v_B = R_WB v_W`

### 3.3 Constants
- Standard gravity magnitude `g = 9.80665 m/s²`

---

## 4. Core State Definition

All core classes live in `src/core/state/`.

### 4.1 Nominal State
Minimal AHRS state (attitude + biases + reference vectors):

- Orientation: `q_WB` (unit quaternion)
- Gyro bias: `b_g ∈ ℝ³` (rad/s)
- Accel bias: `b_a ∈ ℝ³` (m/s²)
- Gravity in world: `g_W ∈ ℝ³` (m/s²)
- Magnetic field in world: `m_W ∈ ℝ³` (tesla)

If extrinsics are needed later, add them as separate classes under
`src/core/state/` and keep them out of the minimal AHRS state.

### 4.2 Error-State Vector
The ESKF error-state is a 15×1 vector:

```
δx = [
  δθ   (3),  small-angle attitude error (rad)
  δb_g (3),  gyro bias error (rad/s)
  δb_a (3),  accel bias error (m/s²)
  δg_W (3),  gravity vector error (m/s²)
  δm_W (3),  magnetic field error (tesla)
]ᵀ
```

- The relationship between nominal and true attitude is:
  `q_true = q_nom ⊗ δq`, where `δq ≈ [1, 0.5 δθ]`

---

## 5. Process Model (Propagation)

All propagation math lives in `src/core/models/process_model.py` and is
used by `filters/eskf.py`.

### 5.1 Gyro Integration
Given a gyro measurement `ω_meas` in body frame:

```
ω = ω_meas - b_g
|ω| = sqrt(ωᵀ ω)
```

Quaternion update over timestep `dt`:

```
if |ω| ≈ 0:
  δq = [1, 0.5 ω dt]
else:
  axis = ω / |ω|
  angle = |ω| dt
  δq = [cos(angle/2), axis * sin(angle/2)]

q_WB ← q_WB ⊗ δq
q_WB ← normalize(q_WB)
```

### 5.2 Bias and Reference Vector Propagation
Biases and reference vectors are modeled as random walks:

```
b_g ← b_g + n_bg * dt
b_a ← b_a + n_ba * dt

g_W ← g_W + n_g * dt
m_W ← m_W + n_m * dt
```

Process noise terms (all zero-mean, white):
- `n_bg` gyro bias drift (rad/s²)
- `n_ba` accel bias drift (m/s³)
- `n_g` gravity drift (m/s³)
- `n_m` mag drift (tesla/s)

### 5.3 Error-State Dynamics (Linearized)
Let `Ω = [ω]×` be the skew-symmetric matrix of `ω`.

```
δθ̇  = -Ω δθ - δb_g - n_gyr
δḃ_g = n_bg
δḃ_a = n_ba
δġ_W = n_g
δṁ_W = n_m
```

The continuous-time system matrix `F` is:

```
F = [
  -Ω   -I    0    0    0
   0    0    0    0    0
   0    0    0    0    0
   0    0    0    0    0
   0    0    0    0    0
]
```

The noise input matrix `G` maps the noise vector
`w = [n_gyr, n_bg, n_ba, n_g, n_m]` into `δẋ`:

```
G = [
  -I  0   0   0   0
   0  I   0   0   0
   0  0   I   0   0
   0  0   0   I   0
   0  0   0   0   I
]
```

### 5.4 Discrete Covariance Propagation
For timestep `dt`:

```
Φ ≈ I + F dt
Q_d ≈ G Q_c Gᵀ dt

P ← Φ P Φᵀ + Q_d
```

`Q_c` is the continuous noise covariance for
`[n_gyr, n_bg, n_ba, n_g, n_m]`.

---

## 6. Measurement Models (Updates)

All measurement models live in `src/core/models/` with update logic in
`filters/eskf.py`.

### 6.1 Accelerometer (Gravity Observation)
Assume the accelerometer measures **specific force** in body frame:

```
a_meas = R_WB g_W + b_a + n_a + a_dyn
```

Where:
- `a_dyn` is unmodeled linear acceleration (treated as noise)
- `n_a` is measurement noise

Use the **gravity observation** model:

```
z = a_meas
h(x) = R_WB g_W + b_a
r = z - h(x)
```

Linearized Jacobian `H_a` for the error-state:

```
H_a = [
  -R_WB [g_W]×   0   I   R_WB   0
]
```

### 6.2 Magnetometer (Field Observation)
Magnetometer measures the Earth field in body frame:

```
m_meas = R_WB m_W + n_m
```

Measurement model:

```
z = m_meas
h(x) = R_WB m_W
r = z - h(x)
```

Linearized Jacobian `H_m`:

```
H_m = [
  -R_WB [m_W]×   0   0   0   R_WB
]
```

### 6.3 Optional Gyro Bias Measurement
If a calibration source provides a gyro bias prior `b_g0` with covariance
`R_bg`, treat it as a measurement:

```
z = b_g0
h(x) = b_g
r = z - h(x)

H_bg = [ 0  I  0  0  0 ]
```

---

## 7. ESKF Update Equations (Explicit)

For any measurement `z` with model `h(x)` and Jacobian `H`:

```
S = H P Hᵀ + R
K = P Hᵀ S⁻¹
δx = K r
P ← (I - K H) P (I - K H)ᵀ + K R Kᵀ
```

### 7.1 Injecting the Error-State
Given `δx = [δθ, δb_g, δb_a, δg_W, δm_W]`:

```
δq = [1, 0.5 δθ]
q_WB ← q_WB ⊗ δq
q_WB ← normalize(q_WB)

b_g ← b_g + δb_g
b_a ← b_a + δb_a

g_W ← g_W + δg_W
m_W ← m_W + δm_W
```

After injection, **reset** the error-state to zero.

---

## 8. Initial Alignment (Bootstrap)

Implement in `src/core/bootstrap/initial_alignment.py`.

### 8.1 Inputs
- A single accelerometer sample `a_meas` (m/s²)
- A single magnetometer sample `m_meas` (tesla)

### 8.2 Steps
1. Normalize gravity direction in body:

```
g_B = -a_meas / |a_meas|
```

2. Compute roll and pitch from gravity:

```
roll  = atan2(g_B.y, g_B.z)

pitch = atan2(-g_B.x, sqrt(g_B.y² + g_B.z²))
```

3. Remove gravity component from magnetometer:

```
m_B = m_meas / |m_meas|

m_h = m_B - (m_B · g_B) g_B
m_h = m_h / |m_h|
```

4. Compute yaw using the horizontal magnetic component:

```
yaw = atan2(-m_h.y, m_h.x)
```

5. Build `q_WB` from roll/pitch/yaw in ZYX order.

### 8.3 Initialize Reference Vectors
- `g_W = [0, 0, g]`
- `m_W` from the initial yaw and `m_B`:

```
m_W = R_WBᵀ m_B
```

---

## 9. Class Responsibilities

### 9.1 `core/state/ahrs_state.py`
- Holds the nominal state
- Serialization and validation helpers

### 9.2 `core/state/ahrs_error_state.py`
- Holds covariance `P` and provides reset/injection helpers

### 9.3 `core/models/process_model.py`
- Implements propagation (`Predict()`)
- Computes `F`, `G`, and discrete propagation of `P`

### 9.4 `core/models/imu_model.py`
- Computes accelerometer residuals and `H_a`
- Provides noise matrix `R_a`

### 9.5 `core/models/mag_model.py`
- Computes magnetometer residuals and `H_m`
- Provides noise matrix `R_m`

### 9.6 `core/filters/eskf.py`
- Orchestrates predict/update with explicit math from Sections 5–7

### 9.7 `core/bootstrap/initial_alignment.py`
- Produces initial `q_WB`, `g_W`, `m_W` from a single IMU+mag sample

### 9.8 `ros/*` and `nodes/ahrs_node.py`
- ROS message handling only
- Zero estimation logic

---

## 10. Required Public APIs

Each class should expose a minimal, ROS-agnostic API.

### `Eskf`
- `Predict(gyro_meas: Vec3, dt: float) -> None`
- `UpdateAccel(accel_meas: Vec3, R: Mat3) -> None`
- `UpdateMag(mag_meas: Vec3, R: Mat3) -> None`
- `GetState() -> AhrsState`

### `InitialAlignment`
- `Compute(a_meas: Vec3, m_meas: Vec3) -> AhrsState`

### `AhrsState`
- `q_WB: Quaternion`
- `b_g: Vec3`
- `b_a: Vec3`
- `g_W: Vec3`
- `m_W: Vec3`

---

## 11. Testing Expectations

- Unit tests for quaternion math and skew matrix
- Unit tests for `H_a`, `H_m`, and covariance propagation
- Simple Monte Carlo test: static IMU, verify convergence of `q_WB`

---

## 12. Dependencies

- Only Python standard library plus the repository’s existing math utilities
- No ROS imports in `src/core`
- Keep APIs pure and deterministic
