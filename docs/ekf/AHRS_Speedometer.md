# AHRS Forward Twist Design Document

This document defines the AHRS-based forward-twist contract for the current
rail-vehicle motion model.

The node consumes mounted body-frame IMU data on `ahrs/imu` plus a zero-velocity
signal on `zupt` / `zupt_flag`. It estimates signed 1D vehicle speed along a
learned forward direction and publishes the corresponding body-frame velocity as
`ahrs/forward_twist`.

The key policy difference from a generic body-frame speedometer is that the
vehicle's forward motion axis is not assumed to equal `base_link +x`. The
vehicle moves only along one fixed forward axis in `base_link`, parameterized by
a learned in-plane yaw angle `forward_yaw`.

This document defines the runtime behavior contract, ROS interfaces, online
learning policy, drift-control policy, and covariance contract for that 1D
forward-twist estimator.

---

## 1. Goals and non-goals

### 1.1 Goals

- estimate signed 1D speed for a rail vehicle that moves only along one forward
  axis in the body frame
- consume mounted `ahrs/imu` in `base_link`
- use all available IMU mean fields when useful, especially orientation,
  angular velocity, and linear acceleration
- learn the forward body-frame axis automatically after boot
- define positive direction from the first accepted motion after boot
- support online learning with checkpoints
- stop learning and discard uncommitted learning evidence when turn detection
  indicates the motion is no longer suitable for axis learning
- use ZUPT for speed drift control
- publish full output covariance with no diagonalization
- persist learned `forward_yaw` to disk for inspection and debugging

### 1.2 Non-goals

- no full 3D velocity estimation
- no lateral velocity estimation
- no global position estimation
- no track-map or curvature estimation
- no reliance on a separate `ahrs/gravity` topic
- no trust in upstream IMU covariance buckets as the speedometer uncertainty
  model
- no loading of persisted `forward_yaw` during startup in the current phase;
  startup always relearns

---

## 2. Motion model

The speedometer uses the rail-vehicle motion prior:

- the vehicle velocity is 1D
- translation lies only along one fixed forward axis in `base_link`
- yaw of the body may change during runtime
- yawing does not invalidate the 1D runtime speed model by itself
- turns do invalidate online axis learning, because curved motion is not
  suitable evidence for learning one fixed forward axis

State of interest:

- scalar signed speed `forward_speed`
- scalar learned forward heading `forward_yaw`

Derived quantity:

- unit forward axis `forward_axis`

Body-frame velocity model:

- `v_B = forward_speed * forward_axis`

where:

- `forward_axis = [cos(forward_yaw), sin(forward_yaw), 0]^T`
- `||forward_axis|| = 1`

By policy:

- `forward_axis` lies in the body x-y plane
- positive speed means motion along the learned forward direction
- negative speed means motion opposite the learned forward direction
- the first accepted motion after boot defines the positive direction

---

## 3. Frame contract

Frames:

- `{W}`: world
- `{B}`: base (`base_link`)

Public frame names:

- `world`
- `base_link`

The node consumes AHRS outputs that are already mounted into `base_link`. It
does not estimate or apply an additional IMU mounting transform. Its learned
quantity is instead the fixed forward direction inside `base_link`.

This is not an IMU extrinsic calibration. It is a forward-axis calibration in
the mounted body frame.

---

## 4. ROS input

### 4.1 Streams

- `ahrs/imu` (`sensor_msgs/Imu`): mounted body-frame IMU stream
- `zupt` (`geometry_msgs/TwistWithCovarianceStamped`): zero-velocity
  observation
- `zupt_flag` (`std_msgs/Bool`): zero-velocity status flag

By policy:

- `ahrs/imu.header.frame_id == "base_link"`
- `zupt_flag` is the primary Boolean stationarity indicator
- `zupt` provides the corresponding zero-velocity measurement and covariance
- the node does not consume a separate `ahrs/gravity` topic

### 4.2 `ahrs/imu` used fields

- `header.stamp` as `t_meas_ns`
- `orientation`
- `angular_velocity` as `ω_B`
- `linear_acceleration` as `a_B`

The node should use all useful IMU mean fields whenever possible.

In particular:

- `orientation` is used to interpret the current mounted body-frame attitude and
  to define the persisted `forward_yaw` convention
- `angular_velocity` is used heavily for turn detection and learning-stop logic
- `linear_acceleration` is used for learning evidence and speed propagation

Covariance policy:

- the upstream `ahrs/imu` covariance fields may be present
- they are not trusted as the speedometer uncertainty model
- they must not be blindly propagated as authoritative process or output
  uncertainty
- the speedometer computes its own output covariance

### 4.3 `zupt` used fields

- `header.stamp`
- `twist.twist.linear`
- `twist.covariance`

Policy:

- the node uses `zupt_flag` to detect stationary updates
- the node uses the `zupt` zero-velocity covariance as the uncertainty of the
  zero-speed correction
- the zero-speed fact is used for scalar speed drift control
- ZUPT is not required for forward-axis learning or checkpoint commit

---

## 5. Learned quantity

### 5.1 Primary learned parameter

The node learns one scalar parameter:

- `forward_yaw`

This is the fixed in-plane yaw rotation in `base_link` that defines the
vehicle's forward direction.

The corresponding unit vector is reconstructed as:

- `forward_axis = [cos(forward_yaw), sin(forward_yaw), 0]^T`

### 5.2 Why `forward_yaw` is the persisted quantity

The learnable geometry is one degree of freedom in the mounted body frame. The
current contract therefore persists the scalar `forward_yaw` rather than a free
3D vector.

Benefits:

- simpler persistence
- simpler inspection and debugging
- explicit in-plane forward-axis convention
- direct reconstruction of `forward_axis`

### 5.3 Persistence policy

Current OASIS persistence convention stores calibration metadata under `~/.ros`,
with install-time symlinks pointing into package config directories. In
particular, `~/.ros/mount_info` is linked to `oasis_control/config/mount_info`
by the existing install script, alongside the similar `imu_info` and
`magnetometer_info` directories.

The forward-twist node should follow that same convention.

Directory:

- `~/.ros/mount_info/`

Recommended filename pattern:

- `forward_twist_<hostname>.yaml`

Example on Falcon:

- `~/.ros/mount_info/forward_twist_falcon.yaml`

Persisted quantity:

- `forward_yaw`

Current runtime policy:

- write `forward_yaw` to disk
- do not load persisted `forward_yaw` on startup
- always relearn `forward_yaw` after boot
- persisted values are for debugging, comparison across runs, and future
  extension

Write policy:

- write only checkpointed, committed values
- never write candidate or uncommitted learning state
- overwrite the same host-specific file for the current machine
- writes should be atomic from the node's point of view

Suggested YAML shape:

```yaml
version: 1
created_unix_ns: <int>
host: <host_id>
estimator: ahrs_forward_twist
valid: true
forward_yaw_rad: <float>
forward_axis:
  - <x>
  - <y>
  - 0.0
fit_sample_count: <int>
checkpoint_count: <int>

---

## 6. Calibration and online learning

### 6.1 Startup convention

After boot:

- AHRS has already produced mounted `ahrs/imu`
- the forward-twist node starts with no trusted learned forward axis
- the first accepted motion after boot is defined as forward by policy

This startup rule resolves the otherwise ambiguous sign of the learned axis.

### 6.2 Online learning model

The node supports online learning with checkpoints.

Maintain:

- `candidate_forward_yaw`
- `committed_forward_yaw`
- uncommitted learning evidence accumulated since the last checkpoint

Policy:

- learning occurs only while motion is acceptable for straight-axis estimation
- checkpoint commits are based on confidence and stability, not on ZUPT alone
- if turn detection fires, uncommitted learning evidence is discarded
- the last committed `forward_yaw` remains valid

### 6.3 Suitable learning evidence

Learning evidence comes from body-frame motion that is consistent with the 1D
straight-axis model.

At minimum, accepted learning periods should satisfy:

- valid `ahrs/imu`
- sufficient motion excitation
- low model mismatch relative to the current candidate axis
- no fused turn detection
- enough accepted evidence for stable candidate estimation

Useful evidence sources include:

- body-frame linear acceleration direction over time
- consistency of repeated forward/back motion along one axis
- agreement of candidate updates across a recent evidence window

### 6.4 Checkpoint commit rules

A checkpoint may be committed whenever the candidate estimate is stable enough.

Typical commit conditions include:

- minimum accepted sample count
- minimum excitation magnitude
- small recent change in `candidate_forward_yaw`
- sufficiently low orthogonal residual
- no fused turn detection

ZUPT may be a convenient checkpoint moment when available, but it is not
required. Long forward/back/forward/back motion without ZUPT should still allow
online learning and checkpoint commits.

### 6.5 Learning stop / discard rules

Learning must stop when fused turn detection indicates the motion is not valid
for forward-axis learning.

When that happens:

- discard uncommitted learning evidence since the last checkpoint
- keep the last committed `forward_yaw`
- continue runtime speed estimation if the committed forward axis is valid

---

## 7. Turn detection

### 7.1 Purpose

Turn detection is used to protect axis learning. It is not, by itself, a reason
to invalidate the runtime 1D speed model.

The train may still have valid 1D body-frame forward speed while the body yaws
through a curve.

### 7.2 Fused turn detection

Turn detection should use fused evidence from both:

- angular velocity
- acceleration behavior

This is preferred over a simple single-threshold rule.

Angular-velocity evidence helps detect turning onset. Acceleration evidence
helps detect motion patterns inconsistent with straight-axis learning.

### 7.3 Runtime meaning

Turn detection affects:

- whether online learning is allowed
- whether uncommitted learning evidence is discarded

Turn detection does not automatically imply:

- invalid runtime speed
- automatic uncertainty inflation
- loss of the committed forward axis

---

## 8. Runtime speed estimation

### 8.1 Scalar speed propagation

The node estimates one scalar speed:

- `forward_speed`

The corresponding body-frame linear velocity is:

- `v_B = forward_speed * forward_axis`

### 8.2 Acceleration projection

The runtime propagation should use the measured body-frame acceleration projected
onto the learned axis:

- `a_forward = forward_axis^T a_B`

where:

- `a_B` is taken from `ahrs/imu.linear_acceleration`

The node should keep the propagation model deterministic and diagnosable.

### 8.3 Drift control with ZUPT

ZUPT is used for scalar speed drift control.

When `zupt_flag` is true:

- apply a zero-speed correction toward `forward_speed = 0`
- use the `zupt` covariance as the uncertainty of that correction
- reduce speed uncertainty appropriately through the zero-speed update

This lets the node control drift without making ZUPT a requirement for online
axis learning.

### 8.4 Runtime behavior during turns

Turns do not automatically invalidate runtime speed propagation.

Policy:

- continue speed estimation using the committed `forward_axis`
- do not inflate speed uncertainty merely because the body is yawing
- let uncertainty growth follow the actual propagation and correction model

If future evidence shows genuine model mismatch, that should be handled through
explicit model-mismatch logic rather than a blanket "turning means worse
uncertainty" rule.

---

## 9. Outputs

### 9.1 Primary output

Recommended primary topic:

- `ahrs/forward_twist`
- type: `geometry_msgs/TwistWithCovarianceStamped`

Contract:

- `header.frame_id = "base_link"`
- `twist.twist.linear = forward_speed * forward_axis`
- `twist.twist.angular` may be left zero by this node unless a wrapper policy is
  added later
- `covariance` is full and must not be diagonalized

### 9.2 Optional debug outputs

Useful optional debug/status outputs include:

- learned `forward_yaw`
- reconstructed `forward_axis`
- candidate vs committed forward-yaw estimates
- learning state
- turn-detection state
- speed sigma for HUD use

---

## 10. Covariance contract

### 10.1 General rule

The node must publish full covariance matrices and must never diagonalize by
convenience.

### 10.2 Upstream IMU covariance policy

The `ahrs/imu` covariance fields may come from generic driver bucket values and
should not be treated as a trustworthy physical uncertainty model for this node.

Therefore:

- do not use upstream IMU covariance as the authoritative speedometer
  uncertainty input
- do not simply rotate or forward those covariance blocks into the
  `forward_twist` output
- compute output covariance from the speedometer's own model

### 10.3 Separation of speed uncertainty and axis uncertainty

Two uncertainty concepts should remain distinct:

- scalar speed uncertainty
- learned forward-axis uncertainty

For the HUD, the primary displayed quantity is:

- `forward_speed ± 1 sigma`

That `1 sigma` should come from the scalar speed uncertainty.

Axis uncertainty is separate and mainly affects calibration confidence or future
extensions to the full twist covariance model.

### 10.4 Output covariance structure

For the current phase, it is acceptable for the dominant contribution to the
published linear-velocity covariance to come from scalar speed uncertainty.

If axis uncertainty is treated as negligible compared with speed uncertainty in
the current phase, then the linear velocity covariance may be approximated from
the projection of the scalar speed variance through `forward_axis`.

This still yields a full `3 x 3` covariance block and must not be diagonalized.

When axis uncertainty is later modeled explicitly, it can be added as an
additional contribution without changing the public contract.

---

## 11. HUD contract

The HUD displays:

- `forward_speed`
- `± 1 sigma`

That `1 sigma` should be the speedometer's scalar speed standard deviation.

The HUD should not derive its uncertainty from an arbitrary norm of the full
twist covariance. The displayed uncertainty is specifically the 1D speed
uncertainty.

---

## 12. Time handling

Definitions:

- `t_meas_ns`: accepted measurement timestamp in integer nanoseconds
- `t_last_ns`: last accepted timestamp

Processing rule:

1. validate incoming data
2. update turn-detection state from IMU mean fields
3. if learning is allowed, update the candidate forward-axis estimate
4. checkpoint candidate state when stability criteria are met
5. project acceleration onto the committed forward axis
6. propagate scalar speed
7. if `zupt_flag` is true, apply zero-speed correction
8. publish `ahrs/forward_twist` and diagnostics

Ordering rule:

- out-of-order samples may be dropped deterministically
- no fixed-lag replay is required
- all acceptance, discard, and checkpoint rules must remain deterministic and
  diagnosable

---

## 13. Diagnostics

Recommended diagnostics fields:

- learning state
- candidate `forward_yaw`
- committed `forward_yaw`
- reconstructed `forward_axis`
- accepted learning sample count
- rejected learning sample count
- discarded uncommitted sample count
- checkpoint commit count
- checkpoint discard count
- fused turn-detection state
- turn-detection trigger count
- current `forward_speed`
- current scalar speed variance
- ZUPT correction count
- last accepted timestamp
- out-of-order drop count
- persistence write status

---

## 14. Parameters

Topics:

- `topics.imu` default `ahrs/imu`
- `topics.zupt` default `zupt`
- `topics.zupt_flag` default `zupt_flag`
- `topics.output_forward_twist` default `ahrs/forward_twist`

Frames:

- `base_frame_id` default `base_link`

Learning policy:

- minimum excitation magnitude
- minimum accepted sample count
- candidate stability threshold
- orthogonal residual threshold
- fused turn-detection thresholds
- checkpoint commit thresholds

Speed policy:

- speed process-noise settings
- ZUPT correction enable/disable
- ZUPT measurement usage policy
- out-of-order sample handling
- covariance publication policy

Persistence policy:

- path for persisted `forward_yaw`
- write-on-checkpoint policy
- startup load disabled by default in the current phase

---

## 15. Policy boundary

This node owns:

- forward-axis learning in `base_link`
- online checkpointed estimation of `forward_yaw`
- signed scalar speed estimation along the learned axis
- ZUPT-based speed drift control
- publication of `ahrs/forward_twist`
- output covariance computed from its own model
- persistence of `forward_yaw` for debugging and inspection

This node does not own:

- IMU mounting calibration
- AHRS attitude publication
- global localization
- track-map estimation
- trust in generic upstream IMU covariance buckets as the node's uncertainty
  model
- startup loading of persisted `forward_yaw` in the current phase
