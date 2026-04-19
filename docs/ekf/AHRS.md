# AHRS Design Document

This document defines the AHRS contract for the current 9-DoF IMU.

The AHRS consumes a fused `sensor_msgs/Imu` stream on `imu` plus a required
gravity-direction observation on `gravity`. The IMU message already includes an
accurate quaternion orientation estimate, so this node does not re-fuse
separate inertial or magnetic sources. Its job is to validate both inputs,
apply the fixed IMU-to-base mounting rotation, check gravity consistency, and
publish ROS-friendly attitude outputs.

The AHRS is the attitude-layer component. It owns mounted IMU/gravity
interpretation and standalone attitude publication, but it does not own global
localization or non-identity `world -> odom` correction. In standalone AHRS
operation, `world -> odom` is identity by policy.

---

## 1. Frame contract

The AHRS publishes the standard TF chain:

`world -> odom -> base_link`

Policy:

- `world` is initialized at startup and remains continuous
- `odom` is identical to `world` in this node
- `world -> odom` is identity for the full session
- `odom -> base_link` carries the mounted base attitude with zero translation

This keeps the TF tree compatible with later localization layers that may own
`world -> odom`.

---

## 2. ROS input

### 2.1 Streams

- `imu` (`sensor_msgs/Imu`): fused IMU sample stream
- `gravity` (`geometry_msgs/AccelWithCovarianceStamped`): gravity-direction
  observation stream

Both streams are part of the AHRS contract. `gravity` is not optional. The
implementation may process them as separate event streams, but the node should
not advertise a degraded "IMU-only" operating mode.

By policy:

- `imu.header.frame_id == "imu_link"`
- `gravity.header.frame_id == "imu_link"`
- `imu` and `gravity` are expected in the same sensor frame

### 2.2 `imu` used fields

- `header.stamp` as `t_meas_ns`
- `header.frame_id` as IMU frame `{I}`
- `orientation` as `q_WI`
- `orientation_covariance` as `Σ_qI`
- `angular_velocity` as `ω_I`
- `angular_velocity_covariance` as `Σ_ωI`
- `linear_acceleration` as `a_I`
- `linear_acceleration_covariance` as `Σ_aI`

Requirements:

- quaternion data must be finite and non-zero norm
- the AHRS normalizes `q_WI` before use
- vectors are interpreted in `{I}`
- full covariances are preserved when valid
- if `orientation_covariance[0] == -1`, orientation covariance is treated as
  unknown per ROS convention

Reject the sample if quaternion or covariance data is non-finite, or if the IMU
frame does not match the expected `imu_link` policy.

### 2.3 `gravity` used fields

- `header.stamp` as `t_meas_ns`
- `header.frame_id` as IMU frame `{I}`
- `accel.accel.linear` as the measured physical gravity vector
- `accel.covariance` as `Σ_g`

Requirements:

- the vector must be finite and non-zero norm
- `gravity.header.frame_id == "imu_link"`
- `gravity.accel.accel.linear` is expressed in `imu_link`
- the vector points down in the direction of gravitational acceleration
- the magnitude is expected to be near `9.81 m/s^2` at rest
- this is a physical gravity vector, not an "up" vector and not a normalized
  unit vector
- the vector is interpreted as a first-class gravity measurement, not a
  propagation input
- the full `3 x 3` covariance is preserved when valid

Reject the sample if the vector or covariance is non-finite, or if the gravity
frame does not match the expected IMU frame policy.

---

## 3. Mounting transform and gravity use

Frames:

- `{W}`: world
- `{O}`: odom
- `{B}`: base (`base_link`)
- `{I}`: IMU (`imu_link`)

The AHRS uses a fixed transform `T_BI` from `imu_link` to `base_link`.

- `T_BI` is supplied by static TF or persisted boot-time mounting calibration
- `T_BI` is not estimated online here
- translation is ignored for this attitude-only contract

The current default runtime source is a launch-managed
`tf2_ros/static_transform_publisher` for `base_link -> imu_link`. The default
launch configuration publishes identity mounting unless a different transform is
configured.

The current default calibration source is the boot-time stationary solve in
`AHRS_Mounting.md`, which uses a `2.0`-second window and fixes unobservable
mounting yaw by policy. How `T_BI` is solved belongs in
`AHRS_Mounting.md`, not this runtime behavior contract.

Let `R_WI` come from `q_WI` and `R_BI` come from `T_BI`.
Then:

- `R_WB = R_BI * R_WI`
- `q_WB = q_BI ⊗ q_WI`
- `ω_B = R_BI * ω_I`
- `a_B = R_BI * a_I`

Covariance mapping uses the same rotation:

- `Σ_qB = R_BI * Σ_qI * R_BIᵀ`
- `Σ_ωB = R_BI * Σ_ωI * R_BIᵀ`
- `Σ_aB = R_BI * Σ_aI * R_BIᵀ`

Because `imu` and `gravity` are both expected in `imu_link`, the same mounting
rotation is used to express the gravity vector and its covariance in `{B}`
before any consistency check:

- `g_B_meas = R_BI * g_I_meas`
- `Σ_gB = R_BI * Σ_g * R_BIᵀ`

The gravity measurement is first-class because it provides an external check on
the fused quaternion after mounting is applied:

- validate that `q_WB` predicts a base-frame gravity direction consistent with
  the measured direction
- improve roll/pitch consistency when the fused quaternion and measured gravity
  begin to diverge
- support mounting consistency checks
- leave a clear hook for future or current online mounting refinement if that
  policy is enabled later

Gravity does not provide absolute yaw observability and the AHRS must not claim
otherwise.

### 3.1 Gravity consistency

Let `g_W` denote the world-frame gravity direction used by policy. OASIS keeps
`g_W = (0, 0, -1)` as the shared direction convention. The AHRS predicts the
corresponding base-frame direction from the mounted attitude:

- `g_B_pred = R_BW * g_W`

The residual is the directional mismatch between `g_B_meas` and `g_B_pred`.
Implementation details are flexible, but the contract is:

- compute the residual in `base_link`
- use full covariance from the gravity message
- use the absolute direction residual as the primary AHRS gate
- publish Mahalanobis distance for diagnostics, but treat it as advisory unless
  covariance trust is explicitly strengthened by policy
- gate or reject measurements with clearly inconsistent direction
- do not silently diagonalize covariance to simplify the test
- keep publishing mounted `ahrs/imu`, `ahrs/odom`, and `odom -> base_link`
  outputs when IMU and mounting are available
- use gravity gating to drive diagnostics and consistency status only
- do not use gravity rejection to invent yaw
- do not suppress mounted outputs when IMU and mounting are available

This is a consistency measurement within the AHRS contract, not a separate
navigation filter.

---

## 4. Time handling

The node is event-driven on accepted `imu` and `gravity` samples.

Definitions:

- `t_meas_ns`: message timestamp in integer nanoseconds
- `t_now_ns`: receipt time for diagnostics
- `t_last_ns`: last accepted sample time

Processing rule:

1. Validate timestamp and numeric fields for the incoming sample
2. Normalize the IMU quaternion when an `imu` sample arrives
3. Resolve required mounting transform(s)
4. Rotate orientation, angular velocity, acceleration, gravity, and known
   covariances into `{B}` as needed
5. Evaluate gravity residuals against the mounted attitude estimate
6. Publish outputs stamped at the accepted measurement time

Ordering rule:

- if `t_meas_ns < t_last_ns`, the message may be dropped deterministically
- no fixed-lag buffer is maintained
- no replay logic is required
- gravity and IMU need not be exactly synchronized, but the implementation must
  make the pairing or latest-sample policy deterministic and diagnosable

---

## 5. Outputs

### 5.1 TF

- `world -> odom`: identity
- `odom -> base_link`: mounted attitude, zero translation

### 5.2 Base-frame IMU output

Recommended primary topic:

- `ahrs/imu`
- type: `sensor_msgs/Imu`
- `header.frame_id = "base_link"`
- `orientation = q_WB`
- `orientation_covariance = Σ_qB` when known
- `angular_velocity = ω_B`
- `angular_velocity_covariance = Σ_ωB` when known
- `linear_acceleration = a_B`
- `linear_acceleration_covariance = Σ_aB` when known

Downstream body-motion consumers should prefer `ahrs/imu` over the raw `imu`
topic. Because `ahrs/imu` is already mounted into `base_link`, consumers such
as tilt and speedometer may treat orientation, angular velocity, and linear
acceleration as body-frame signals and should not compensate
`imu_link -> base_link` mounting internally.

### 5.3 Gravity-facing output or status

The AHRS may publish a gravity status or debug topic, but the minimum contract
is that gravity consistency results are reflected in diagnostics and any
accept/reject policy.

### 5.4 Optional odometry wrapper

Downstream consumers may still want `nav_msgs/Odometry`, so it should be a thin
wrapper around the same attitude sample:

- pose position = zero
- pose orientation = `q_WB`
- twist angular = `ω_B`
- linear velocity = unknown or explicit zero by policy

### 5.5 Diagnostics

Recommended diagnostics fields:

- accepted sample count
- dropped sample count
- invalid quaternion count
- invalid gravity count
- transform lookup failure count
- gravity residual norm
- gravity Mahalanobis distance when covariance is available
- gravity gating or rejection count
- mounting consistency failure count
- last accepted timestamp
- clock-jump reset count

If the implementation applies correction or refinement hooks from gravity, it
should also report whether the hook was merely monitored, gated in, or rejected.
For the current lightweight runtime, gravity gating is diagnostic-only: mounted
AHRS outputs continue publishing while the diagnostics report whether the latest
gravity consistency check gated in or was rejected.

---

## 6. Parameters

Frames:

- `world_frame_id` default `world`
- `odom_frame_id` default `odom`
- `base_frame_id` default `base_link`
- `imu_frame_id` default `imu_link`

Topics:

- `topics.imu` default `imu`
- `topics.gravity` default `gravity`
- `topics.output_imu` default `ahrs/imu`
- `topics.output_odom` default `ahrs/odom`

Policy:

- out-of-order sample handling
- clock-jump reset threshold
- required transform source for `T_BI`
- gravity residual gating thresholds
- gravity/IMU pairing policy when timestamps do not match exactly
