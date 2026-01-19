# AHRS Structure Spec

This document defines the **directory layout**, **class boundaries**, and
**mathematical contracts** for a clean-room AHRS reimplementation. It is
written so each component can be implemented independently with no assumed
Kalman filter background. The goal is to rebuild the AHRS as a set of
**small, ROS-agnostic classes** under `src/` with a thin ROS adapter in
`nodes/`.

---

## 1. Goals and scope

- Provide a **modular, testable** AHRS architecture with clear separation
  between core math/estimation and ROS integration
- Keep all estimation logic **ROS-free** (no ROS types, no logging)
- Fully document **all math** needed for each component so implementation is
  deterministic
- Define **stable, message-driven APIs**: plain structs, arrays, and spans

Non-goals:

- No tuning guidance beyond required parameters
- No sensor driver code

---

## 2. Proposed directory layout

```
src/
  ahrs/
    core/
      ahrs_estimator.py
      ahrs_state.py
      ahrs_error_state.py
      ahrs_config.py
      ahrs_types.py
      ahrs_timeline.py
      ahrs_bootstrap.py
      ahrs_inject.py

    math/
      linalg.py
      quat.py
      se3.py
      conversions.py

    model/
      imu_model.py
      mag_model.py
      gravity_model.py
      earth_field_model.py

    filter/
      predict.py
      update_gyro.py
      update_accel.py
      update_mag.py
      error_ekf.py

    io/
      imu_packet.py
      mag_packet.py

nodes/
  ahrs_node.py
  ahrs_ros_conversions.py
  ahrs_ros_clock.py
  ahrs_publishers.py
```

Notes:

- `src/ahrs/...` contains **all algorithmic code** with no ROS references
- `nodes/` contains **only** ROS glue: parameters, subscriptions, QoS,
  logging, conversions, and publishers
- `io/` holds plain input structs that mirror ROS messages without depending
  on ROS types

---

## 3. Common notation

### 3.1 Frames

- `{W}`: world frame
- `{B}`: body frame
- `{I}`: IMU measurement frame
- `{M}`: magnetometer frame

### 3.2 Rotation and vectors

- Quaternion `q_WB` rotates a vector from `{B}` to `{W}`
- Rotation matrix `R_WB = R(q_WB)`
- Skew operator:

```
[ω]× = [[  0  -ωz  ωy],
        [ ωz   0  -ωx],
        [-ωy  ωx   0 ]]
```

### 3.3 State and error-state

Nominal state components:

- `p_WB ∈ ℝ³` position of body in world
- `v_WB ∈ ℝ³` velocity of body in world
- `q_WB` unit quaternion
- `b_g ∈ ℝ³` gyro bias
- `b_a ∈ ℝ³` accel bias
- `A_a ∈ ℝ^{3×3}` accel scale/misalignment matrix
- `g_W ∈ ℝ³` gravity vector in world
- `m_W ∈ ℝ³` earth magnetic field in world
- `T_BI ∈ SE(3)` IMU-to-body extrinsic (rotation + translation)
- `T_BM ∈ SE(3)` mag-to-body extrinsic (rotation + translation)

Error-state (additive for vectors, small-angle for orientation):

```
δx = [δp, δv, δθ, δb_g, δb_a, vec(δA_a), δg, δm, δρ_BI, δθ_BI,
      δρ_BM, δθ_BM]
```

- `δθ` is a 3-vector small-angle perturbation of `q_WB`
- `δρ_*` is the translation perturbation for extrinsics
- `δθ_*` is the rotation perturbation for extrinsics

---

## 4. Core components and APIs

### 4.1 `core/ahrs_types.py`

Define plain data structures used throughout the core. All fields must be
documented with units and interpretation.

Minimum structs:

- `ImuSample`
  - `t: float` seconds
  - `omega: Vec3` rad/s measured in `{I}`
  - `accel: Vec3` m/s² measured in `{I}`
  - `cov_omega: Mat3` (rad/s)²
  - `cov_accel: Mat3` (m/s²)²

- `MagSample`
  - `t: float` seconds
  - `mag: Vec3` tesla measured in `{M}`
  - `cov_mag: Mat3` tesla²

- `AhrsOutput`
  - `t: float` seconds
  - `q_WB: Quaternion`
  - `p_WB: Vec3`
  - `v_WB: Vec3`
  - `b_g: Vec3`
  - `b_a: Vec3`
  - `A_a: Mat3`

### 4.2 `core/ahrs_state.py`

Holds the **nominal state**. Provide methods:

- `ApplyErrorState(delta: ErrorState)`
- `Reset()`

**Error-state application math**:

- Position/velocity/bias updates are additive:
  - `p_WB ← p_WB + δp`
  - `v_WB ← v_WB + δv`
  - `b_g ← b_g + δb_g`
  - `b_a ← b_a + δb_a`
  - `A_a ← A_a + δA_a`
  - `g_W ← g_W + δg`
  - `m_W ← m_W + δm`

- Quaternion perturbation using small-angle `δθ`:
  - `δq = [1, 0.5 δθ]` (approximate unit quaternion)
  - `q_WB ← Normalize(δq ⊗ q_WB)`

- Extrinsics on SE(3):
  - `R ← Exp([δθ]×) R`
  - `t ← t + δρ`

### 4.3 `core/ahrs_error_state.py`

Defines the error-state vector and covariance matrix `P`. Provide:

- `Dim()`
- `Zero()`
- `Inject(delta)` that resets error to zero after applying to state

### 4.4 `core/ahrs_config.py`

Configuration values with explicit units:

- `sigma_bg: float` rad/s/√s
- `sigma_ba: float` m/s²/√s
- `sigma_gyro: float` rad/s
- `sigma_accel: float` m/s²
- `sigma_mag: float` tesla
- `sigma_g: float` m/s²/√s
- `sigma_m: float` tesla/√s
- `R_mag_min: Mat3`, `R_mag_max: Mat3`
- `alpha_R_mag: float` [0, 1]

### 4.5 `core/ahrs_timeline.py`

Manages fixed-lag replay. API:

- `PushImu(sample)`
- `PushMag(sample)`
- `Tick(t_now)` returns list of events to process

No math beyond ordering and buffer logic.

### 4.6 `core/ahrs_bootstrap.py`

Initialization policy:

- On first valid IMU sample, set:
  - `q_WB` from accel-only tilt if needed
  - `g_W` magnitude = 9.80665 m/s², direction from accel
  - `m_W` from first mag sample after tilt correction

**Tilt from accel**:

- Given `a_meas` in `{I}` and `R_BI` (IMU to body):
  - `a_B = R_BI a_meas`
  - Assume `a_B ≈ -R_BW g_W` where `g_W = [0, 0, 9.80665]`
  - Solve for `R_WB` such that `R_WBᵀ g_W = -a_B`

### 4.7 `core/ahrs_inject.py`

Handles error-state injection and covariance stabilization:

- After each update, apply `δx` to nominal state
- Reset error state to zero
- Enforce `P = 0.5 (P + Pᵀ)`
- Clamp eigenvalues to `>= eps` if needed

---

## 5. Math utilities

### 5.1 `math/quat.py`

Functions:

- `Normalize(q)`
- `Multiply(q1, q2)`
- `FromSmallAngle(δθ)`
- `ToRotation(q)`
- `Integrate(q, ω, dt)`

**Quaternion integration**:

- Angular rate in body frame `{B}`: `ω_B`
- Update rule (first-order):

```
q_{k+1} = Normalize(q_k ⊗ [1, 0.5 ω_B dt])
```

### 5.2 `math/linalg.py`

Provide matrix helpers:

- `Skew(ω)`
- `ExpSO3(δθ)` for small-angle rotation
- `ClampSPD(P, P_min, P_max)`

### 5.3 `math/se3.py`

Helpers for SE(3):

- `ApplyTransform(T, v)`
- `Compose(T1, T2)`
- `ExpSE3(δρ, δθ)`

### 5.4 `math/conversions.py`

Frame conversions that avoid ROS types:

- `RotateVector(q, v)`
- `RotateCovariance(R, P)`

---

## 6. Measurement models

### 6.1 `model/imu_model.py`

**Raw IMU**:

- Gyro measurement in `{I}`:

```
ω_meas = ω_true + b_g + n_g
```

- Accel measurement in `{I}`:

```
a_meas = A_a (a_true - b_a) + n_a
```

Where:

- `n_g ~ N(0, Σ_ω)`
- `n_a ~ N(0, Σ_a)`

**Specific force model**:

```
a_true = R_BIᵀ R_WBᵀ (a_W - g_W)
```

For stationary assumptions, `a_W ≈ 0`.

### 6.2 `model/mag_model.py`

Mag measurement in `{M}`:

```
m_meas = R_MBᵀ R_WBᵀ m_W + n_m
```

Where:

- `R_MB` is rotation from body to mag frame
- `n_m ~ N(0, R_mag)`

### 6.3 `model/gravity_model.py`

Treat gravity vector `g_W` as slowly varying random walk:

```
ḡ_W = n_gW
```

`n_gW ~ N(0, σ_g² I)`

### 6.4 `model/earth_field_model.py`

Treat magnetic field `m_W` as slowly varying random walk:

```
ṁ_W = n_mW
```

`n_mW ~ N(0, σ_m² I)`

---

## 7. Filter pipeline

### 7.1 `filter/predict.py`

Continuous-time nominal dynamics:

```
ṗ_WB = v_WB
ṽ_WB = R_WB a_B + g_W
q̇_WB = 0.5 Ω(ω_B) q_WB
ḃ_g = n_bg
ḃ_a = n_ba
Ȧ_a = n_Aa
```

Where:

- `a_B` and `ω_B` are IMU-corrected measurements in `{B}`
- `Ω(ω)` is the quaternion rate matrix

Discrete-time propagation (Euler):

```
p_{k+1} = p_k + v_k dt
v_{k+1} = v_k + (R_WB a_B + g_W) dt
q_{k+1} = Integrate(q_k, ω_B, dt)
```

**Corrected IMU**:

```
ω_B = R_BI (ω_meas - b_g)

a_B = R_BI A_a (a_meas - b_a)
```

### 7.2 `filter/error_ekf.py`

Error-state propagation:

```
δẋ = F δx + G w
Ṗ = F P + P Fᵀ + G Q Gᵀ
```

Discretize with first-order approximation:

```
Φ = I + F dt
Q_d = G Q Gᵀ dt
P ← Φ P Φᵀ + Q_d
```

Document every block of `F` and `G` in terms of the state ordering. Provide
exact expressions derived from the nominal model and measurement equations
above. Each block should be a short, explicit formula (no assumed knowledge).

### 7.3 `filter/update_gyro.py`

Gyro update (optional if using only for prediction). If included:

```
z = ω_meas
ẑ = R_IB ω_B + b_g
ν = z - ẑ
```

`H` maps error-state to `ν` by linearizing w.r.t `b_g` and `δθ_BI`.

### 7.4 `filter/update_accel.py`

Accel measurement update for gravity direction:

```
z = a_meas
ẑ = A_a^{-1} R_IB (R_WBᵀ g_W)
ν = z - ẑ
```

Use when `|a_meas|` is near `|g|` to avoid dynamic acceleration corruption.

### 7.5 `filter/update_mag.py`

Mag update:

```
z = m_meas
ẑ = R_MBᵀ R_WBᵀ m_W
ν = z - ẑ
```

Adaptive `R_mag` update:

```
R_mag ← clamp_SPD((1 - α) R_mag + α (ν νᵀ - H P Hᵀ), R_min, R_max)
```

---

## 8. High-level estimator class

### 8.1 `core/ahrs_estimator.py`

Responsibilities:

- Own `AhrsState`, `ErrorState`, and `Config`
- Process samples in chronological order
- Call `predict` with IMU samples
- Call measurement updates with mag samples
- Perform error injection and covariance stabilization

Suggested API:

- `PushImu(ImuSample)`
- `PushMag(MagSample)`
- `StepTo(t)` returns `AhrsOutput`

---

## 9. ROS integration (thin layer)

`nodes/ahrs_node.py` responsibilities:

- Subscribe to ROS topics
- Convert ROS messages into `ImuSample` and `MagSample`
- Forward samples to core estimator
- Publish ROS outputs
- Handle parameters and logging

No math in this layer.

---

## 10. Implementation order

1. `math/` utilities (quat, linalg, se3, conversions)
2. `core/` types and state containers
3. `model/` measurement models
4. `filter/` predict + error EKF
5. `filter/` update steps
6. `core/` estimator orchestration
7. `nodes/` ROS adapter

---

## 11. Validation plan (minimal)

- Unit-test `quat.Integrate` with constant-rate rotations
- Validate `ExpSO3` small-angle consistency
- Simulate static IMU: confirm gravity alignment update convergence
- Feed constant mag field: confirm yaw convergence

