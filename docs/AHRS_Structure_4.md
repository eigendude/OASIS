# AHRS Structure Specification

## Purpose

This document defines a clean-room structure for the AHRS subsystem and
provides a complete, self-contained math specification for each component. The
intent is to restart implementation from scratch with small, reusable classes
and a clear separation between core algorithms and ROS integration.

The spec is written so each component can be implemented independently with no
prior Kalman filter knowledge beyond what is described here.

## Design Goals

- **Core first**: algorithms and data transforms live in `src/ahrs/...` and are
  ROS-agnostic.
- **Thin integration**: ROS nodes and message adapters live in `nodes/` or
  `ros/` and only convert between messages and core data types.
- **Small classes**: each file implements one focused class or set of related
  free functions.
- **Explicit data**: timestamps, sample intervals, and configuration are passed
  explicitly.
- **Documented math**: every constant and equation has a short explanation of
  meaning, units, and derivation.

## Proposed Directory Structure

```
oasis_control/
  oasis_control/
    localization/
      ahrs/
        src/
          ahrs/
            core/
              ahrs_filter.py
              ahrs_state.py
              ahrs_update.py
              ahrs_types.py
            math/
              quat.py
              rotation.py
              linalg.py
            models/
              gyro_model.py
              accel_model.py
              mag_model.py
            sensors/
              imu_sample.py
              mag_sample.py
            utils/
              clock.py
              timeline.py
        ros/
          ahrs_ros_node.py
          ahrs_ros_publishers.py
          ahrs_ros_params.py
          ahrs_ros_conversions.py
        tests/
          test_quat.py
          test_ahrs_update.py
```

Notes:

- `src/ahrs/core/` holds the high-level filter orchestration and state.
- `src/ahrs/math/` holds math utilities and quaternion/rotation utilities.
- `src/ahrs/models/` holds sensor models and error-state Jacobians.
- `src/ahrs/sensors/` holds typed sensor samples and validation.
- `src/ahrs/utils/` holds clocks and timing utilities.
- `ros/` is reserved for ROS2 integration, nodes, parameters, and publishers.

## Common Data Types

All core classes must use the same minimal, reusable types. Public structures
must be documented with field comments (meaning, units, expected range, and how
used).

### Vector and Matrix Types

Use lightweight types that do not depend on numpy. A recommended approach is
small tuples or lightweight dataclasses with fixed-size lists. The following
shapes are used throughout:

- `Vector3`: `[x, y, z]` in units specified by the context
- `Matrix3`: 3x3 matrix of floats, row-major
- `Quat`: quaternion `[w, x, y, z]` with unit length

### Time and Sample Types

- `Timestamp`: float seconds since an arbitrary epoch
- `DeltaTime`: float seconds between samples, strictly positive

### Sensor Samples

- `ImuSample`
  - `timestamp` (s): sample time
  - `gyro_rad_s` (rad/s): angular velocity in body frame
  - `accel_m_s2` (m/s^2): acceleration in body frame

- `MagSample`
  - `timestamp` (s): sample time
  - `mag_uT` (uT): magnetic field in body frame

## Frames and Conventions

- **Body frame**: sensor frame, right-handed
- **World frame**: navigation frame, right-handed
- **Quaternion**: `q = [w, x, y, z]` represents rotation from body to world
- **Rotation**: apply `v_world = R(q) * v_body`

## Mathematical Foundations

This section provides the equations used by the filter. Each component in the
implementation must adhere to these equations exactly.

### Quaternion Algebra

**Quaternion normalization**

Given `q = [w, x, y, z]`, normalize as:

```
q_norm = q / ||q||

||q|| = sqrt(w^2 + x^2 + y^2 + z^2)
```

**Quaternion multiplication**

Given `p = [pw, px, py, pz]` and `q = [qw, qx, qy, qz]`:

```
p ⊗ q = [
  pw*qw - px*qx - py*qy - pz*qz,
  pw*qx + px*qw + py*qz - pz*py,
  pw*qy - px*qz + py*qw + pz*px,
  pw*qz + px*py - py*px + pz*qw
]
```

**Quaternion from angular velocity**

Given angular velocity `ω = [wx, wy, wz]` in rad/s and time step `dt`:

```
|ω| = sqrt(wx^2 + wy^2 + wz^2)

if |ω| > 0:
  axis = ω / |ω|
  angle = |ω| * dt
  dq = [cos(angle/2), axis_x*sin(angle/2), axis_y*sin(angle/2), axis_z*sin(angle/2)]
else:
  dq = [1, 0, 0, 0]
```

Use `q_next = q ⊗ dq` and then normalize.

**Rotation matrix from quaternion**

Let `q = [w, x, y, z]`. The rotation matrix `R(q)` is:

```
R = [
  [1 - 2(y^2 + z^2),     2(x*y - w*z),     2(x*z + w*y)],
  [    2(x*y + w*z), 1 - 2(x^2 + z^2),     2(y*z - w*x)],
  [    2(x*z - w*y),     2(y*z + w*x), 1 - 2(x^2 + y^2)]
]
```

### Gravity and Magnetic Field References

**Gravity reference**

- `g_world = [0, 0, 9.80665]` m/s^2
- Expected accelerometer reading (no linear acceleration):
  `accel_body_expected = R(q)^T * g_world`

**Magnetic reference**

- Provide a reference vector in world frame: `m_world` in uT
- Expected magnetometer reading:
  `mag_body_expected = R(q)^T * m_world`

## Filter Architecture

The filter is an **error-state Extended Kalman Filter (ES-EKF)** operating on a
quaternion orientation and gyro bias. It has a nominal state and a small error
state.

### Nominal State

- `q` : orientation quaternion (body to world)
- `b_g` : gyro bias in rad/s

### Error State

Small error vector `x = [δθ, δb_g]`:

- `δθ` : 3x1 small-angle orientation error in radians
- `δb_g` : 3x1 gyro bias error in rad/s

The error-state relates to the nominal state via:

```
q_true ≈ q_nominal ⊗ [1, 0.5*δθ]

b_g_true = b_g_nominal + δb_g
```

Where `[1, 0.5*δθ]` denotes a small-angle quaternion with scalar part 1 and
vector part `0.5*δθ`.

### Continuous-Time Process Model

The nominal state evolves as:

```
ω_corr = ω_meas - b_g

q_dot = 0.5 * q ⊗ [0, ω_corr]

b_g_dot = 0
```

The error-state dynamics are:

```
δθ_dot = -[ω_corr]× * δθ - I * δb_g - n_g

δb_g_dot = n_b
```

Where:

- `[ω]×` is the 3x3 skew-symmetric cross-product matrix
- `n_g` is gyro noise (rad/s)
- `n_b` is gyro bias random walk (rad/s^2)

Skew-symmetric matrix:

```
[ω]× = [
  [ 0, -wz,  wy],
  [ wz,  0, -wx],
  [-wy, wx,  0]
]
```

### Discrete-Time State Transition

For small `dt`, approximate the error-state transition as:

```
F = [
  [I - [ω_corr]×*dt,   -I*dt],
  [      0,            I    ]
]
```

Process noise covariance for gyro noise and bias random walk:

```
Q = [
  [σ_g^2 * dt * I,             0],
  [            0,     σ_b^2 * dt * I]
]
```

Where:

- `σ_g` is gyro noise density (rad/s/√Hz)
- `σ_b` is bias random walk density (rad/s^2/√Hz)

### Measurement Models

The measurement updates use accelerometer and magnetometer data when valid.

#### Accelerometer Update

Assume the measured acceleration is mostly gravity. The measurement equation is:

```
z_a = accel_body_meas

h_a(q) = R(q)^T * g_world

r_a = z_a - h_a(q)
```

Linearize around the nominal state. For small `δθ`:

```
h_a(q_true) ≈ h_a(q_nominal) + H_a * δθ

H_a = -R(q)^T * [g_world]×
```

The full measurement Jacobian for the error-state is:

```
H = [H_a, 0]
```

Use measurement covariance:

```
R_a = σ_a^2 * I
```

Where `σ_a` is accelerometer noise standard deviation (m/s^2).

#### Magnetometer Update

Measurement equation:

```
z_m = mag_body_meas

h_m(q) = R(q)^T * m_world

r_m = z_m - h_m(q)
```

Linearization:

```
H_m = -R(q)^T * [m_world]×

H = [H_m, 0]
```

Measurement covariance:

```
R_m = σ_m^2 * I
```

Where `σ_m` is magnetometer noise standard deviation (uT).

### EKF Update Equations

Given error-state covariance `P`:

```
S = H * P * H^T + R

K = P * H^T * S^-1

δx = K * r

P = (I - K * H) * P
```

Inject the error-state correction into nominal state:

```
q = q ⊗ [1, 0.5*δθ]

b_g = b_g + δb_g
```

Then reset the error-state to zero and update covariance with the reset
Jacobian:

```
G = [
  [I - 0.5*[δθ]×, 0],
  [0, I]
]

P = G * P * G^T
```

### Validity Gates

Each measurement update should be optionally gated by a residual norm test:

```
if ||r|| > gate_threshold:
  skip update
```

Use separate gates for accelerometer and magnetometer.

## Component Specifications

Each component listed below should map to a class or set of functions in the
proposed directory layout.

### core/ahrs_types.py

**Purpose**: central definitions for common data types used in the core.

**Contents**:

- `Vector3`, `Matrix3`, `Quat` simple containers
- `AhrsConfig` configuration container
- `AhrsState` nominal state container

**Required fields**:

`AhrsConfig` must include:

- `gyro_noise` (rad/s/√Hz)
- `gyro_bias_noise` (rad/s^2/√Hz)
- `accel_noise` (m/s^2)
- `mag_noise` (uT)
- `gravity_m_s2` (m/s^2)
- `mag_world_uT` (uT)
- `accel_gate` (m/s^2)
- `mag_gate` (uT)

### core/ahrs_state.py

**Purpose**: maintain the nominal state and error covariance.

**Responsibilities**:

- Hold `q`, `b_g`, `P`
- Provide getters and setters
- Provide `Reset()` to default values

### core/ahrs_update.py

**Purpose**: implement EKF prediction and measurement update steps.

**Public API**:

- `Predict(state, imu_sample, dt, config)`
- `UpdateAccel(state, imu_sample, config)`
- `UpdateMag(state, mag_sample, config)`

Each update must follow the math in this document.

### core/ahrs_filter.py

**Purpose**: orchestration class combining prediction and update logic.

**Responsibilities**:

- Manage last timestamp and compute `dt`
- Decide when to apply accel and mag updates
- Provide a clean update API for the node layer

**Public API**:

- `Update(imu_sample, mag_sample_opt)`
- `GetState()`
- `Reset()`

### math/quat.py

**Purpose**: quaternion operations.

**Required functions**:

- `Normalize(q)`
- `Multiply(p, q)`
- `FromAngularVelocity(omega, dt)`
- `ToRotationMatrix(q)`

### math/rotation.py

**Purpose**: rotation utilities.

**Required functions**:

- `Skew(omega)`
- `RotateVector(q, v)`
- `RotateVectorInverse(q, v)`

### math/linalg.py

**Purpose**: minimal linear algebra utilities (3x3, 6x6).

**Required functions**:

- `MatMul(A, B)` for 3x3 and 6x6
- `MatVec(A, v)`
- `Transpose(A)`
- `Identity(n)`
- `Inverse3(A)`
- `Inverse6(A)`

### models/gyro_model.py

**Purpose**: gyro noise and bias random walk models.

**Required functions**:

- `ComputeProcessNoise(dt, config)` returns `Q`
- `ComputeStateTransition(omega, dt)` returns `F`

### models/accel_model.py

**Purpose**: accelerometer measurement model and Jacobian.

**Required functions**:

- `ExpectedAccel(q, gravity)`
- `AccelJacobian(q, gravity)`
- `AccelResidual(z, q, gravity)`

### models/mag_model.py

**Purpose**: magnetometer measurement model and Jacobian.

**Required functions**:

- `ExpectedMag(q, mag_world)`
- `MagJacobian(q, mag_world)`
- `MagResidual(z, q, mag_world)`

### sensors/imu_sample.py

**Purpose**: sensor sample definitions and validation.

**Required functions**:

- `ValidateImuSample(sample)` for NaN/inf and reasonable ranges

### sensors/mag_sample.py

**Purpose**: magnetometer sample definitions and validation.

**Required functions**:

- `ValidateMagSample(sample)` for NaN/inf and reasonable ranges

### utils/clock.py

**Purpose**: time utilities for monotonic timestamps.

**Required functions**:

- `DeltaTime(prev, current)`

### utils/timeline.py

**Purpose**: optional sample buffering or time alignment.

**Required behavior**:

- Provide a simple FIFO queue with time-based lookup

### ros/ahrs_ros_node.py

**Purpose**: ROS2 node wrapper for `AhrsFilter`.

**Responsibilities**:

- Subscribe to IMU and magnetometer topics
- Convert messages into core types
- Publish orientation and bias outputs
- Manage parameters and QoS

## Implementation Notes

- All algorithm code must avoid ROS dependencies.
- All public structs must include brief field docs.
- Wrap comments at 80 chars.
- For math constants, use a comment above the line documenting units and
  derivation.
- Keep functions small and focused.

## Validation Plan

- Unit tests for quaternion math and matrix utilities
- Unit tests for prediction and update steps using known inputs
- End-to-end test for convergence with synthetic data

## Migration Plan

1. Implement math utilities (`quat.py`, `rotation.py`, `linalg.py`).
2. Implement data types and config (`ahrs_types.py`).
3. Implement gyro model and process equations (`gyro_model.py`).
4. Implement accel and mag models (`accel_model.py`, `mag_model.py`).
5. Implement EKF update logic (`ahrs_update.py`).
6. Implement state wrapper (`ahrs_state.py`).
7. Implement filter orchestration (`ahrs_filter.py`).
8. Implement ROS integration (`ros/ahrs_ros_node.py`).
9. Add tests and validation.
