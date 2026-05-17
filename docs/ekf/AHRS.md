# AHRS Design Document

This document defines the AHRS contract for the current 9-DoF IMU.

The AHRS consumes three raw driver streams: `imu`, `imu_gravity`, and
`gravity`. It consumes both gravity-removed and gravity-included IMU forms,
while `gravity` remains a required physical gravity-vector observation. The
IMU messages already include accurate quaternion orientation estimates, so
this node does not re-fuse separate inertial or magnetic sources. Its job is
to validate inputs, apply the fixed IMU-to-base mounting rotation, check
gravity consistency, and publish ROS-friendly attitude outputs.

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

- `imu` (`sensor_msgs/Imu`): fused IMU sample stream with gravity-removed
  linear acceleration
- `imu_gravity` (`sensor_msgs/Imu`): fused orientation and gyro with
  gravity-included acceleration
- `gravity` (`geometry_msgs/AccelWithCovarianceStamped`): gravity-direction
  observation stream

All streams are part of the AHRS contract. `gravity` is not optional. The
implementation may process them as separate event streams, but the node should
not advertise a degraded "IMU-only" operating mode.

Required BNO -> AHRS input streams use RELIABLE QoS with bounded keep-last
history. AHRS is a required pipeline stage, so raw `imu`, `imu_gravity`, and
`gravity` should not be treated as disposable live telemetry.

These streams are independent event streams. They are not required to have the
same cadence, and implementations must not assume `imu` and `imu_gravity`
publish together.

By policy:

- `imu.header.frame_id == "imu_link"`
- `imu_gravity.header.frame_id == "imu_link"`
- `gravity.header.frame_id == "imu_link"`
- `imu`, `imu_gravity`, and `gravity` are expected in the same sensor frame

### 2.2 `imu` used fields

- `header.stamp` as `t_meas_ns`
- `header.frame_id` as IMU frame `{I}`
- `orientation` as driver-frame quaternion `q_IW`
- `orientation_covariance` as `Σ_qI`
- `angular_velocity` as `ω_I`
- `angular_velocity_covariance` as `Σ_ωI`
- `linear_acceleration` as `a_I`
- `linear_acceleration_covariance` as `Σ_aI`

Requirements:

- `imu.linear_acceleration` is gravity-removed acceleration
- quaternion data must be finite and non-zero norm
- the AHRS normalizes the incoming driver quaternion and canonicalizes it to
  `q_WI = q_IW*` before applying mounting
- vectors are interpreted in `{I}`
- full covariances are preserved when valid
- if `orientation_covariance[0] == -1`, orientation covariance is treated as
  unknown per ROS convention
- upstream IMU orientation covariance is driver-owned policy data before AHRS
  sees the sample
- AHRS preserves upstream orientation covariance unless the runtime output
  policy explicitly replaces the published orientation covariance with the
  gravity-observable attitude model described below

Reject the sample if quaternion or covariance data is non-finite, or if the IMU
frame does not match the expected `imu_link` policy.

### 2.3 `imu_gravity` used fields

- `header.stamp` as `t_meas_ns`
- `header.frame_id` as IMU frame `{I}`
- `orientation` as driver-frame quaternion `q_IW`
- `orientation_covariance` as `Σ_qI`
- `angular_velocity` as `ω_I`
- `angular_velocity_covariance` as `Σ_ωI`
- `linear_acceleration` as gravity-included acceleration `a_I + g_I`
- `linear_acceleration_covariance` as `Σ_aI`

Requirements:

- `imu_gravity` is the preferred high-rate gravity-included IMU stream for
  VIO/SLAM-style consumers
- its acceleration cadence may differ from `imu` because the two streams have
  different acceleration semantics
- quaternion and vector fields must be finite
- full covariances are preserved when valid
- `imu_gravity.header.frame_id == "imu_link"`
- orientation, angular velocity, and acceleration are expressed in `imu_link`
- this stream uses calibrated acceleration including gravity, not the
  gravity-removed linear acceleration used by `imu`

### 2.4 `gravity` used fields

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
- `gravity` cadence is independent of `imu` and `imu_gravity`

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

- `T_BI` is supplied by boot-time mounting calibration or an explicit external
  calibration source
- `T_BI` is not estimated online here
- translation is ignored for this attitude-only contract

The current default runtime source is the boot-time stationary solve in
`AHRS_Mounting.md`. `ahrs_node` learns the fixed quaternion `q_BI`, which
rotates vectors from `imu_link` into `base_link`, from a short gravity window
at boot. That same `q_BI` is then broadcast on the TF edge with
parent=`base_link`, child=`imu_link`, so `lookup_transform(base_link, imu_link)`
returns the runtime mounting used by AHRS.

Session yaw policy:

- physical mounting remains the fixed IMU-to-base transform `T_BI`
- AHRS still does not claim gravity estimated mounting yaw
- once mounting is available and the first valid mounted attitude sample is
  accepted, AHRS captures that initial mounted yaw as a runtime-only session
  yaw zero
- subsequent published AHRS orientations subtract that session yaw offset for
  the duration of the node run
- this session yaw convention does not change the physical
  `base_link -> imu_link` TF or redefine `q_BI`

Let `R_WI` come from the canonicalized `q_WI = q_IW*` and let `R_BI` come from
`T_BI`.
Then:

- `R_WB = R_BI * R_WI`
- `q_WB = q_BI ⊗ q_WI`
- `ω_B = R_BI * ω_I`
- `a_B = R_BI * a_I`

After mounting is applied, AHRS defines a runtime session yaw convention for
published world-facing orientation outputs:

- let `ψ_init` be the yaw of the first valid mounted attitude `q_WB`
- let `q_zero` be the pure yaw quaternion for `-ψ_init`
- published orientation outputs use `q_pub = q_zero ⊗ q_WB`
- this keeps roll/pitch from the mounted attitude while expressing yaw
  relative to the startup mounted direction

Angular-velocity and linear-acceleration covariance mapping use the same
rotation:

- `Σ_ωB = R_BI * Σ_ωI * R_BIᵀ`
- `Σ_aB = R_BI * Σ_aI * R_BIᵀ`

Policy boundary:

- AHRS still treats driver-provided `orientation_covariance` as meaningful
  upstream sensor policy data at the validation boundary
- the mounted runtime output no longer blindly republishes that upstream
  orientation covariance block
- instead, `ahrs/imu.orientation_covariance` is owned by the AHRS output
  contract and is shaped to reflect the current gravity-observable
  roll/pitch model plus separately modeled yaw uncertainty
- this is a deliberate downstream reinterpretation for the published AHRS
  attitude contract, not an accidental byproduct of mounting rotation

Because `imu`, `imu_gravity`, and `gravity` are all expected in `imu_link`,
the same mounting rotation is used to express the gravity vector and its
covariance in `{B}` before any consistency check:

- `g_B_meas = R_BI * g_I_meas`
- `Σ_gB = R_BI * Σ_g * R_BIᵀ`

By design, the raw `imu` topic may show non-zero roll/pitch in `imu_link` when
the sensor is physically mounted at a tilt. That is expected. The AHRS contract
is that the mounted `ahrs/imu` output in `base_link` removes that fixed tilt.

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
otherwise. The boot mounting solve only measures roll/pitch alignment and keeps
physical mounting yaw fixed by policy. AHRS may still publish yaw relative to a
runtime session yaw zero captured from the initial mounted heading, but that is
a startup heading convention rather than a physical yaw calibration.

### 3.1 Published orientation covariance semantics

The published `ahrs/imu.orientation_covariance` is the AHRS-facing attitude
uncertainty contract used later by EKF consumers and HUD/debug tools.

Current semantics:

- the roll/pitch block reflects gravity-observable attitude uncertainty in
  `base_link`
- gravity constrains roll and pitch
- gravity does not observe yaw
- yaw variance in the published covariance is handled separately from the
  gravity-derived roll/pitch model

For roll and pitch, AHRS uses the mounted gravity vector and its mounted
covariance, then propagates that covariance through the same OASIS roll/pitch
mapping used elsewhere in the stack:

- `roll = atan2(-g_y, -g_z)`
- `pitch = atan2(g_x, sqrt(g_y^2 + g_z^2))`

The local `2 x 2` roll/pitch covariance is approximated by first-order
propagation:

- `Σ_roll,pitch ~= J * Σ_gB * Jᵀ`

where `J` is the Jacobian of that roll/pitch mapping with respect to the
base-frame gravity vector. This is intentionally more exact than the earlier
shared conservative tilt-style variance approximation.

Consequences:

- roll variance and pitch variance may differ
- roll/pitch cross-covariance may be nonzero
- the published `3 x 3` orientation covariance may therefore contain a full
  symmetric roll/pitch block rather than only diagonal terms
- yaw cross-covariances remain zero in this model because gravity does not
  observe heading

For yaw, AHRS publishes a separate uncertainty signal derived from recent yaw
stability on the mounted attitude stream. That yaw variance is not claimed to
come from gravity. This separation is intentional and should be preserved in
downstream interpretation.

### 3.2 Gravity consistency

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

The node is event-driven on accepted `imu`, `imu_gravity`, and `gravity`
samples.

Definitions:

- `t_meas_ns`: message timestamp in integer nanoseconds
- `t_now_ns`: receipt time for diagnostics
- `t_last_ns`: last accepted sample time for the relevant stream

Upstream drivers may deliver samples in bursts. AHRS must treat `header.stamp`
as measurement time and `t_now_ns` only as receipt/diagnostic time. Pairing
with latest samples is allowed, but the policy must be deterministic and
diagnosable.

Processing rule:

1. Validate timestamp and numeric fields for the incoming sample
2. Normalize the IMU quaternion when an `imu` or `imu_gravity` sample arrives
3. Resolve required mounting transform(s)
4. Rotate orientation, angular velocity, acceleration, gravity, and known
   covariances into `{B}` as needed
5. Evaluate gravity residuals against the mounted attitude estimate
6. Publish outputs stamped at the accepted measurement time

Ordering rule:

- if `t_meas_ns < t_last_ns`, the message may be dropped deterministically
- no fixed-lag buffer is maintained
- no replay logic is required
- `imu`, `imu_gravity`, and `gravity` need not be exactly synchronized, but
  the implementation must make the pairing or latest-sample policy
  deterministic and diagnosable

---

## 5. Outputs

### 5.1 TF

- `world -> odom`: identity
- `odom -> base_link`: mounted attitude, zero translation
- the published yaw on `odom -> base_link` is relative to the initial mounted
  direction captured for this node run

### 5.2 Base-frame IMU output

Recommended primary topic:

- `ahrs/imu`
- type: `sensor_msgs/Imu`
- `header.frame_id = "base_link"`
- `orientation = q_pub = q_zero ⊗ q_WB`
- AHRS defines the initial mounted yaw at startup as session yaw zero, so
  published yaw is relative to that startup direction for the duration of the
  node run
- `orientation_covariance = Σ_qB` when known
- if `imu.orientation_covariance[0] == -1`, `ahrs/imu` preserves the ROS
  "unknown orientation covariance" sentinel
- `angular_velocity = ω_B`
- `angular_velocity_covariance = Σ_ωB` when known
- `linear_acceleration = a_B`
- `linear_acceleration_covariance = Σ_aB` when known

Downstream body-motion consumers should prefer `ahrs/imu` over the raw `imu`
topic. Because `ahrs/imu` is already mounted into `base_link`, consumers such
as speedometer may treat orientation, angular velocity, and linear
acceleration as body-frame signals and should not compensate
`imu_link -> base_link` mounting internally.

The `ahrs/imu.orientation_covariance` topic contract remains full mounted AHRS
attitude covariance. Downstream consumers must not reinterpret that matrix as a
tilt-only uncertainty product.

`ahrs/imu` follows upstream `imu` stream semantics and may publish at a
different cadence than `ahrs/imu_gravity`.

EKF propagation should default to `ahrs/imu`. Do not feed both `ahrs/imu` and
`ahrs/imu_gravity` as independent propagation inputs.

### 5.3 Base-frame gravity-included IMU output

- `ahrs/imu_gravity`
- type: `sensor_msgs/Imu`
- `header.frame_id = "base_link"`
- `orientation`, `angular_velocity`, and `linear_acceleration` are the complete
  upstream `imu_gravity` sample mounted into `base_link`
- orientation, angular velocity, and linear acceleration covariances are
  rotated into `base_link`
- output timestamp matches the upstream `imu_gravity` sample timestamp
- `ahrs/imu_gravity` follows upstream `imu_gravity` measurement timestamps and
  should not be downsampled to `imu` cadence merely for synchronization

### 5.4 Gravity-facing output or status

The AHRS publishes mounted physical gravity on `ahrs/gravity` as
`geometry_msgs/AccelWithCovarianceStamped` in `base_link`. EKF consumers should
use it only as a gravity measurement update or consistency check, not as
propagation acceleration.

Gravity consistency results are also reflected in diagnostics and any
accept/reject policy. Gravity consistency uses the latest accepted gravity
measurement under the documented pairing policy.

The downstream-facing `ahrs/imu`, `ahrs/imu_gravity`, and `ahrs/gravity`
streams use RELIABLE QoS with bounded keep-last history for EKF and replay
consumers. Live-only debug streams may remain BEST_EFFORT when losing samples
does not change estimator state.

### 5.5 Optional odometry wrapper

Downstream consumers may still want `nav_msgs/Odometry`, so it should be a thin
wrapper around the same attitude sample:

- pose position = zero
- pose orientation = `q_pub = q_zero ⊗ q_WB`
- pose yaw is expressed relative to the startup session yaw zero
- pose orientation covariance block reuses the same rotated `Σ_qB` when known
- twist angular = `ω_B`
- linear velocity = unknown or explicit zero by policy

### 5.6 Diagnostics

Recommended diagnostics fields:

- last accepted `imu` timestamp
- last accepted `imu_gravity` timestamp
- last accepted `gravity` timestamp
- accepted sample count per stream
- dropped sample count per stream
- invalid quaternion count
- invalid gravity count
- transform lookup failure count
- latest stream age or pairing age
- out-of-order count per stream
- gravity residual norm
- gravity Mahalanobis distance when covariance is available
- gravity gating or rejection count
- mounting consistency failure count
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
- `topics.imu_gravity` default `imu_gravity`
- `topics.gravity` default `gravity`
- `topics.output_imu` default `ahrs/imu`
- `topics.output_imu_gravity` default `ahrs/imu_gravity`
- `topics.output_gravity` default `ahrs/gravity`
- `topics.output_odom` default `ahrs/odom`

Policy:

- out-of-order sample handling
- clock-jump reset threshold
- required transform source for `T_BI`
- gravity residual gating thresholds
- `gravity`, `imu`, and `imu_gravity` pairing policy when timestamps do not
  match exactly
