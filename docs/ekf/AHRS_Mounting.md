# AHRS Mounting Calibration

This document specifies IMU mounting calibration for the current fused IMU.

The node estimates the fixed rotation from `imu_link` to `base_link`,
publishes that transform, and persists it for the AHRS and EKF.

This document is the calibration contract for solving fixed `T_BI`. It does
not define the runtime AHRS behavior contract or the AHRS code layout.

---

## 1. Goals and non-goals

### 1.1 Goals

- learn mounting automatically at boot
- use a short stationary boot window with deterministic averaging and windowing
- estimate the fixed mounting rotation `R_BI`
- publish `base_link -> imu_link` as TF with zero translation
- persist the solved rotation in a deterministic file format
- use gravity as a first-class calibration input
- solve only the observable tilt-alignment part of `R_BI` in the default boot
  mode

### 1.2 Non-goals

- no full 3-DoF mounting solve from a short stationary boot window
- no gravity-based recovery of mounting yaw
- no full navigation state estimation
- no online bias or scale calibration output
- no translation estimation

---

## 2. ROS input

### 2.1 Stream

- `imu` (`sensor_msgs/Imu`): fused IMU sample stream
- `gravity` (`geometry_msgs/AccelWithCovarianceStamped`): gravity-direction
  observation stream

Both streams are required.

Default policy:

- mounting calibration runs automatically at boot
- the default calibration window is `2.0` seconds
- both streams are expected in `imu_link`
- the boot window must be stationary or low angular-rate by policy

### 2.2 `imu` used fields

- `header.stamp`
- `header.frame_id` as IMU frame `{I}`
- `orientation`, `orientation_covariance`
- `angular_velocity`
- `linear_acceleration`

Requirements:

- quaternion must be finite and non-zero norm
- the solver normalizes the quaternion before use
- `imu.header.frame_id` is expected to be `imu_link`

### 2.3 `gravity` used fields

- `header.stamp`
- `header.frame_id` as IMU frame `{I}`
- `accel.accel`
- `accel.covariance`

Requirements:

- the vector must be finite and non-zero norm
- `gravity.header.frame_id` is expected to be `imu_link`
- full covariance is preserved when valid
- covariance is not diagonalized by convenience

The mounting solver uses a deterministic boot windowing policy. Pairing,
averaging, and any reject decisions must remain deterministic and diagnosable.

---

## 3. Frames and TF contract

Frames:

- `{B}`: base (`base_link`)
- `{I}`: IMU (`imu_link`)

Estimated quantity:

- `R_BI`: rotation from IMU frame to base frame

Published transform:

- `T_BI = (R_BI, 0)`

Translation is fixed to zero and is never estimated.

TF direction:

- parent: `base_link`
- child: `imu_link`

So:

- `v_B = R_BI v_I`

Persisted quaternions must declare ordering explicitly.

---

## 4. Calibration method

The default mounting mode is an automatic boot-time solve over a stationary
`2.0`-second window.

The solver collects fused `imu` and `gravity` samples during that boot window
and estimates one fixed transform:

- `T_BI = (R_BI, 0)`

The boot solver is observability-limited.

From a short stationary window with fused IMU attitude and gravity:

- roll and pitch alignment between `imu_link` and `base_link` are observable
- the mounting yaw component is not observable from gravity in this mode

The default contract is therefore:

- estimate the tilt-alignment part of `R_BI`
- fix the unobservable yaw component by policy
- use zero relative yaw between `imu_link` and `base_link` by default

Equivalently, the solver aligns roll and pitch between `imu_link` and
`base_link` and sets mounting yaw to zero unless additional heading
information is intentionally introduced in a future mode.

The exact optimization method is implementation-defined, but the public
contract is one fixed rotation `R_BI` and the corresponding fixed transform
`T_BI = (R_BI, 0)`.

Gravity is first-class in this calibration because it helps:

- validate stationary windows
- compare predicted gravity direction under a candidate tilt alignment against
  the measured gravity direction
- reject inconsistent windows where fused attitude and measured gravity disagree
- score the boot-window tilt solution with covariance-aware residuals

The calibration remains rotation-only. It does not estimate translation and it
does not become a full navigation estimator. Full 3-DoF mounting estimation is
outside the current default contract.

### 4.1 Acceptance rules

A sample window is usable when:

- quaternion data is valid
- gravity data is valid
- the boot window spans the configured `2.0`-second default, unless overridden
- the IMU is sufficiently stationary or low angular-rate for deterministic
  averaging
- measured gravity is consistent with the fused attitude over the accepted
  window
- enough valid samples remain after rejection to support a stable tilt solve

Within an accepted window, full covariance from `imu` and `gravity` should be
preserved when valid. When vectors are mapped through a candidate `R_BI`, the
solver should rotate or otherwise transport the corresponding covariance
explicitly instead of diagonalizing it.

### 4.2 Stability rules

The solution is stable when:

- the accepted boot window yields a bounded tilt solution with deterministic
  output
- the gravity residual and fused-attitude residual remain within configured
  thresholds across the accepted window
- the yaw component is fixed by policy rather than inferred from unobservable
  gravity information

---

## 5. ROS interface

### 5.1 Node name

- `ahrs_node`

### 5.2 Inputs

- `imu` (`sensor_msgs/Imu`)
- `gravity` (`geometry_msgs/AccelWithCovarianceStamped`)

### 5.3 Outputs

- TF for `base_link -> imu_link`
- calibration result topic or service
- persisted calibration file
- diagnostics for accepted windows, rejected windows, and boot calibration
  status
- gravity residual or consistency summary in calibration diagnostics

These outputs define the solved mounting used later by AHRS and EKF. They do
not imply that this calibration node owns runtime attitude publication or
localization outputs.

---

## 6. Persistence format

The saved result should include:

- IMU frame name
- base frame name
- quaternion with explicit ordering
- timestamp of calibration
- optional fit metrics

Example shape:

```yaml
imu:
  parent_frame: base_link
  child_frame: imu_link
  quaternion_wxyz: [1.0, 0.0, 0.0, 0.0]
```

---

## 7. Parameters

Topics:

- `topics.imu` default `imu`
- `topics.gravity` default `gravity`

Frames:

- `base_frame_id` default `base_link`
- `imu_frame_id` default `imu_link`

Calibration policy:

- boot calibration window duration default `2.0` seconds
- stationary or low angular-rate acceptance thresholds
- minimum valid sample count within the boot window
- fused-attitude and gravity consistency thresholds
- yaw-fix policy for unobservable mounting yaw, default zero relative yaw
- output file path
