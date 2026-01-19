# AHRS Structure Spec

## Purpose
Define a clean, class-based AHRS architecture with clear separation between:

- **Core algorithms** (math + state + update rules)
- **Integration glue** (ROS interfaces, parameters, topics)

This document is a detailed specification for refactoring the existing flat
module layout into a structured, reusable set of components. The intent is for
implementation to follow this spec step-by-step without requiring additional
Kalman filter background.

## High-Level Goals

- Keep core AHRS logic **ROS-agnostic** and fully testable in isolation
- Provide **stable, message-driven APIs** that map cleanly to sensor messages
- Keep math documented and parameterized with explicit units and meaning
- Use **small, reusable classes** rather than large, monolithic modules
- Support incremental implementation (each part stands alone)

## Proposed Directory Layout

```
oasis_control/
  src/
    localization/
      ahrs/
        core/
          ahrs_filter.py
          ahrs_state.py
          ahrs_types.py
          ahrs_params.py
          ahrs_clock.py
          ahrs_timeline.py

          math/
            ahrs_linalg.py
            ahrs_quat.py
            ahrs_conversions.py

          model/
            ahrs_predict.py
            ahrs_update_core.py
            ahrs_update_gyro.py
            ahrs_update_accel.py
            ahrs_update_mag.py
            ahrs_error_state.py
            ahrs_inject.py

          integration/
            ahrs_publishers.py
            ahrs_ros_msgs.py
            ahrs_ros_conversions.py
            ahrs_ros_clock.py

        bootstrap/
          ahrs_bootstrap.py

  nodes/
    ahrs_node.py

  docs/
    AHRS_Structure.md
```

### Notes

- **core/**: Generic, ROS-agnostic, state + filter orchestration
- **math/**: Linear algebra + quaternion math + conversions
- **model/**: Error-state EKF update/predict steps
- **integration/**: ROS-specific message conversions + publishers
- **bootstrap/**: CLI or helper routines for calibration or initialization
- **nodes/**: Thin ROS node wrapper

## System Model Overview

### Reference Frames

- **Body frame**: IMU sensor frame
- **Navigation frame**: Local tangent frame (ENU or NED; must be specified)

This spec assumes a **right-handed ENU frame** unless noted otherwise.
If NED is chosen instead, the axis directions and gravity sign must be updated
consistently across all math.

### State Definition

The AHRS maintains a nominal state and an error state.

#### Nominal State

- **Orientation**: unit quaternion, `q_bn`, mapping body → navigation
- **Gyro bias**: vector `b_g`, in rad/s
- **Accel bias**: vector `b_a`, in m/s²
- **Mag bias** (optional): vector `b_m`, in µT or normalized units

#### Error State (EKF)

The error state captures small perturbations of the nominal state:

```
δx = [ δθ, δb_g, δb_a, δb_m ]
```

- `δθ`: small-angle rotation error (rad), 3x1
- `δb_g`: gyro bias error (rad/s), 3x1
- `δb_a`: accel bias error (m/s²), 3x1
- `δb_m`: mag bias error (µT), 3x1

The covariance `P` is defined over the error state.

## Core Math and Models

### Quaternion Basics

- Quaternion `q = [w, x, y, z]`
- Rotation matrix `R(q)` maps body to navigation
- Quaternion multiplication is used for integrating angular velocity

#### Quaternion Integration

Given gyro reading `ω_meas`:

```
ω = ω_meas - b_g
```

The quaternion derivative is:

```
q_dot = 0.5 * Ω(ω) * q
```

Where:

```
Ω(ω) = [ 0, -ωx, -ωy, -ωz
         ωx,  0,  ωz, -ωy
         ωy, -ωz, 0,  ωx
         ωz,  ωy, -ωx, 0 ]
```

Discrete integration (small dt):

```
q_next = normalize(q + q_dot * dt)
```

### Gravity Model

Gravity vector in navigation frame (ENU):

```
g_n = [0, 0, -g]
```

Where:

- `g = 9.80665 m/s²` (standard gravity)

Predicted accel measurement in body frame:

```
a_pred = R(q)^T * g_n + b_a
```

### Magnetometer Model

Assume a fixed local magnetic field vector in navigation frame `m_n`:

```
m_n = [m_x, m_y, m_z]
```

Predicted magnetometer measurement in body frame:

```
m_pred = R(q)^T * m_n + b_m
```

The local field vector may be normalized to unit length or use µT. The
choice must be consistent with measured units and noise covariances.

## EKF Error-State Model

### Error-State Dynamics

Nominal dynamics driven by gyro integration. Error-state dynamics are linearized.

Small-angle error dynamics:

```
δθ_dot = -[ω]_x * δθ - δb_g - n_g
```

Bias random-walk models:

```
δb_g_dot = n_bg
δb_a_dot = n_ba
δb_m_dot = n_bm
```

Where:

- `[ω]_x` is the skew-symmetric matrix of ω
- `n_g` is gyro noise
- `n_bg`, `n_ba`, `n_bm` are bias random walk noises

### Discrete-Time State Transition

For small `dt`, the error-state transition matrix `F` and process noise matrix
`Q` are formed using first-order approximation:

```
Φ ≈ I + F * dt
Q_d ≈ G * Q_c * G^T * dt
```

Where:

- `F` is the continuous-time Jacobian of the error dynamics
- `G` maps continuous noise into state
- `Q_c` is continuous noise covariance

## Measurement Updates

### Gyro Update

Gyro is typically used in propagation only. It does not update the EKF directly
unless a bias estimation step is implemented via pseudo-measurement.

### Accel Update

Measurement residual:

```
r_a = a_meas - a_pred
```

Jacobian with respect to small-angle errors:

```
H_a = [ -R(q)^T * [g_n]_x , 0, I, 0 ]
```

Where:

- `I` corresponds to accel bias error
- `[g_n]_x` is the skew matrix of gravity in navigation frame

### Mag Update

Residual:

```
r_m = m_meas - m_pred
```

Jacobian:

```
H_m = [ -R(q)^T * [m_n]_x , 0, 0, I ]
```

### EKF Update Equations

Given residual `r`, measurement matrix `H`, and measurement noise `R`:

```
S = H * P * H^T + R
K = P * H^T * S^-1
δx = K * r
P = (I - K * H) * P
```

### Error Injection

Apply `δx` to nominal state:

```
q <- q ⊗ δq
b_g <- b_g + δb_g
b_a <- b_a + δb_a
b_m <- b_m + δb_m
```

Where small-angle error is converted to a quaternion:

```
δq = [1, 0.5*δθ]^T (then normalized)
```

Reset the error state after injection.

## Component Specifications

Each component below is a standalone class or module with a well-defined API.
All core components must avoid ROS dependencies.

### core/ahrs_types.py

Defines all public data structures:

- `ImuSample`
  - `timestamp` (float seconds)
  - `gyro` (rad/s, 3-vector)
  - `accel` (m/s², 3-vector)
- `MagSample`
  - `timestamp` (float seconds)
  - `mag` (µT or unit, 3-vector)
- `AhrsState`
  - `orientation` (unit quaternion)
  - `gyro_bias` (rad/s)
  - `accel_bias` (m/s²)
  - `mag_bias` (µT or unit)
- `AhrsConfig`
  - noise and filter parameters with explicit units

All fields must include docstrings with units, meaning, and expected range.

### core/ahrs_params.py

Holds configuration defaults and validation:

- Standard gravity constant
- Default noise covariances
- Default initial covariance
- Validation checks (e.g., positive variances)

### core/ahrs_state.py

Encapsulates nominal state + covariance:

- `state` (AhrsState)
- `P` (covariance matrix)

Provides:

- `reset()`
- `set_state()`
- `get_state()`

### core/ahrs_filter.py

Main filter class:

- Owns `AhrsState` + covariance
- `predict(imu_sample, dt)`
- `update_accel(imu_sample)`
- `update_mag(mag_sample)`
- `get_state()`

No ROS types or logging.

### core/ahrs_clock.py

Minimal clock interface:

- `now()` returns float seconds

### core/ahrs_timeline.py

Tracks sensor sample ordering and dt computation:

- Handles timestamp jitter
- Computes dt safely
- Rejects out-of-order samples

### math/ahrs_linalg.py

Small matrix utilities:

- 3x3/6x6/9x9 operations
- Skew-symmetric matrix
- Symmetric covariance enforcement

### math/ahrs_quat.py

Quaternion operations:

- Multiply, normalize, invert
- Convert small-angle to quaternion
- Quaternion to rotation matrix

### math/ahrs_conversions.py

- Euler ↔ quaternion conversions
- Frame convention helpers

### model/ahrs_predict.py

Implements the propagation step:

- Integrate quaternion
- Propagate covariance via `Φ` and `Q_d`

### model/ahrs_update_core.py

Shared EKF update routine:

- Accepts residual, H, R
- Computes K, δx, updated P

### model/ahrs_update_gyro.py

Optional bias update using gyro as pseudo-measurement.

### model/ahrs_update_accel.py

Accel measurement update:

- Computes predicted accel
- Computes residual + Jacobian
- Calls update core

### model/ahrs_update_mag.py

Mag measurement update:

- Computes predicted mag
- Computes residual + Jacobian
- Calls update core

### model/ahrs_error_state.py

Defines error state indexing:

- Indices for δθ, δb_g, δb_a, δb_m
- Helper methods to pack/unpack δx

### model/ahrs_inject.py

Applies error-state corrections to nominal state:

- Updates quaternion and biases
- Resets error state

### integration/ahrs_ros_msgs.py

ROS message types mapping:

- `sensor_msgs/Imu`
- `sensor_msgs/MagneticField`

### integration/ahrs_ros_conversions.py

- Convert ROS messages → `ImuSample` / `MagSample`
- Convert `AhrsState` → `nav_msgs/Odometry`

### integration/ahrs_ros_clock.py

ROS clock wrapper implementing `core/ahrs_clock` interface

### integration/ahrs_publishers.py

Handles ROS publishers + topic setup

### bootstrap/ahrs_bootstrap.py

Helper routines:

- Load initial biases
- Load magnetic field vector

## Parameter Definition Table

All parameters must specify units and expected ranges.

| Name | Meaning | Units | Typical Range |
|------|---------|-------|----------------|
| `sigma_g` | Gyro noise std dev | rad/s/√Hz | 1e-4 to 1e-2 |
| `sigma_bg` | Gyro bias RW std dev | rad/s²/√Hz | 1e-6 to 1e-3 |
| `sigma_a` | Accel noise std dev | m/s²/√Hz | 1e-3 to 1e-1 |
| `sigma_ba` | Accel bias RW std dev | m/s³/√Hz | 1e-5 to 1e-2 |
| `sigma_m` | Mag noise std dev | µT/√Hz | 1e-2 to 1 |
| `sigma_bm` | Mag bias RW std dev | µT/s/√Hz | 1e-4 to 1e-1 |
| `g` | Gravity magnitude | m/s² | 9.80665 |
| `m_n` | Local mag field | µT or unit | site-specific |

## Implementation Order (Suggested)

1. `ahrs_types.py`
2. `ahrs_params.py`
3. `ahrs_linalg.py`, `ahrs_quat.py`, `ahrs_conversions.py`
4. `ahrs_state.py`
5. `ahrs_predict.py`
6. `ahrs_update_core.py`
7. `ahrs_update_accel.py`
8. `ahrs_update_mag.py`
9. `ahrs_inject.py`
10. `ahrs_filter.py`
11. ROS integration modules

## Validation and Testing Ideas

- Quaternion normalization test
- Covariance symmetry test
- Known static case: only gravity, zero gyro
- Known magnetic heading with fixed m_n

