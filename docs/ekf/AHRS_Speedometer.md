# AHRS Forward Yaw Speedometer Design Document

This document defines the AHRS-based forward-yaw speedometer contract for the
current rail-vehicle motion model.

The node consumes mounted AHRS IMU data on `ahrs/imu` plus a stationary-twist
signal on `zupt` / `zupt_flag`. It estimates signed 1D vehicle speed along a
learned forward direction on the gravity-leveled body surface and publishes the
corresponding forward-twist product as `ahrs/forward_twist`.

The key policy difference from a generic body-frame speedometer is that the
vehicle's forward motion axis is not assumed to equal `base_link +x`, and the
speedometer does not learn a free 3D axis in `base_link`. AHRS already owns
gravity and mounted attitude. The speedometer therefore operates on a flat,
gravity-defined body surface and learns only one scalar geometric parameter:

- `forward_yaw`

The output remains a vector-valued twist. Its linear component is constrained to
the two directions defined by the learned forward axis on that flat surface, and
its angular component includes the yaw angular velocity owned by this node.

This document defines the runtime behavior contract, ROS interfaces, startup
load policy, learning policy, drift-control policy, and covariance contract for
that yaw-only forward-speed estimator. It replaces the deferred-yaw-uncertainty
language in the earlier draft with an explicit uncertainty model for both speed
and yaw.

---

## 1. Goals and non-goals

### 1.1 Goals

- estimate signed 1D speed for a rail vehicle that moves only along one forward
  direction on a gravity-leveled body surface
- consume mounted `ahrs/imu`
- use AHRS attitude ownership to define a flat surface for motion estimation
- learn only the forward yaw on that flat surface
- avoid learning roll or pitch in the speedometer
- use all useful IMU mean fields when helpful, especially orientation,
  angular velocity, and linear acceleration
- estimate and publish yaw angular velocity in the output twist
- use ZUPT for scalar speed drift control
- publish full output covariance with no diagonalization
- keep linear `z` variance ideally zero, while remaining numerically stable
- persist learned `forward_yaw` to disk for inspection and reuse
- load persisted `forward_yaw` on startup when a valid file exists
- use the loaded `forward_yaw` immediately as the authoritative committed value
- disable further yaw learning for that run when startup load succeeds

### 1.2 Non-goals

- no full 3D velocity estimation
- no lateral velocity estimation
- no global position estimation
- no track-map or curvature estimation
- no learning of IMU mounting
- no learning of roll or pitch in the speedometer
- no reliance on a separate `ahrs/gravity` topic
- no trust in upstream IMU covariance buckets as the speedometer uncertainty
  model
- no startup relearning when a valid persisted `forward_yaw` file exists
- no continued online yaw learning after a successful startup load

---

## 2. Motion model

The speedometer uses the rail-vehicle motion prior:

- the vehicle linear velocity is 1D
- translation lies only along one fixed forward direction on a flat,
  gravity-defined body surface
- AHRS already provides mounted attitude information needed to define that
  surface
- the speedometer learns only the in-plane yaw of the rail direction
- yawing of the vehicle body during runtime does not automatically invalidate
  the scalar speed model
- turns do invalidate or gate learning evidence, because curved motion is not
  suitable for learning one fixed straight-line yaw direction

State of interest:

- scalar signed speed `forward_speed`
- scalar learned forward heading `forward_yaw`
- scalar yaw angular velocity `yaw_rate`

Associated uncertainty state:

- scalar speed variance `var_forward_speed_mps2`
- scalar forward-yaw variance `var_forward_yaw_rad2`
- scalar yaw-rate variance `var_yaw_rate_rads2`

Derived quantities:

- unit flat-surface forward axis `forward_axis_flat`
- unit flat-surface lateral axis `lateral_axis_flat`

Flat-surface velocity model:

- `v_B = forward_speed * forward_axis_flat`

where:

- `forward_axis_flat = [cos(forward_yaw), sin(forward_yaw), 0]^T`
- `lateral_axis_flat = [-sin(forward_yaw), cos(forward_yaw), 0]^T`
- `||forward_axis_flat|| = 1`
- `||lateral_axis_flat|| = 1`
- both vectors are expressed in `base_link`

By policy:

- `forward_axis_flat` lies in the `base_link` x-y plane
- positive speed means motion along the learned forward direction
- negative speed means motion opposite the learned forward direction
- the speedometer learns yaw only; it does not estimate any out-of-plane motion
  axis
- the output linear twist is always a vector, but its mean is constrained to the
  one-dimensional subspace spanned by `forward_axis_flat`

---

## 3. Frame contract

Frames:

- `{W}`: world
- `{B}`: mounted body frame (`base_link`)

Public frame names:

- `world`
- `base_link`

AHRS owns gravity interpretation and mounted attitude. The speedometer uses AHRS
attitude to define the flat motion surface, but the estimator state and output
remain expressed in `{B}` / `base_link`.

The speedometer does not estimate:

- IMU extrinsic calibration
- mounting rotation
- roll
- pitch

The learned quantity is instead the in-plane yaw angle of the rail direction in
`base_link`.

This is therefore not a 3D body-axis calibration. It is a yaw-only horizontal
motion-direction calibration.

---

## 4. ROS input

### 4.1 Streams

- `ahrs/imu` (`sensor_msgs/Imu`): mounted body-frame IMU stream
- `zupt` (`geometry_msgs/TwistWithCovarianceStamped`): stationary body-twist
  observation
- `zupt_flag` (`std_msgs/Bool`): stationarity status flag

By policy:

- `ahrs/imu.header.frame_id == "base_link"`
- `zupt_flag` is the primary Boolean stationarity indicator
- `zupt` provides the corresponding stationary-twist measurement and covariance
- the node does not consume a separate `ahrs/gravity` topic

### 4.2 `ahrs/imu` used fields

- `header.stamp` as `t_meas_ns`
- `orientation`
- `angular_velocity` as `ω_B`
- `linear_acceleration` as `a_B`

The node should use all useful IMU mean fields whenever possible.

In particular:

- `orientation` is used to define the gravity-leveled surface and the persisted
  `forward_yaw` convention
- `angular_velocity` is used for turn detection, learning-stop logic, and
  runtime yaw angular velocity output
- `linear_acceleration` is mapped onto the gravity-leveled surface before
  learning or speed propagation

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
- the node uses the relevant linear covariance entry from `zupt` as the
  uncertainty of the scalar zero-speed correction
- the zero-speed fact is used for scalar speed drift control
- ZUPT is not required for forward-yaw learning or checkpoint commit when fresh
  learning is enabled

---

## 5. Learned quantity

### 5.1 Primary learned parameter

The node learns one scalar parameter:

- `forward_yaw`

This is the fixed in-plane yaw rotation in `base_link` that defines the
vehicle's forward rail direction.

The corresponding unit vector is reconstructed as:

- `forward_axis_flat = [cos(forward_yaw), sin(forward_yaw), 0]^T`

### 5.2 Why `forward_yaw` is the persisted quantity

The learnable geometry is one degree of freedom on the gravity-leveled body
surface. The current contract therefore persists the scalar `forward_yaw`
rather than a free 3D vector.

Benefits:

- simpler persistence
- simpler inspection and debugging
- explicit flat-surface forward-direction convention
- direct reconstruction of `forward_axis_flat`
- alignment with the actual physical rail-motion constraint

### 5.3 Persistence policy

Current OASIS persistence convention stores calibration metadata under `~/.ros`,
with install-time symlinks pointing into package config directories. In
particular, `~/.ros/mount_info` is linked to `oasis_control/config/mount_info`
by the existing install script, alongside the similar `imu_info` and
`magnetometer_info` directories.

The forward-yaw speedometer follows that same convention.

Directory:

- `~/.ros/mount_info/`

Recommended filename pattern:

- `forward_twist_<hostname>.yaml`

Example on Falcon:

- `~/.ros/mount_info/forward_twist_falcon.yaml`

Persisted quantity:

- `forward_yaw`

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
confidence: <float>
residual: <float>
score: <float>
uncertainty_forward_yaw_rad: <float>
loaded_startup_capable: <bool>
last_update_reason: <string>
````

Write policy:

* write only committed values
* never write candidate or uncommitted learning state
* overwrite the same host-specific file for the current machine
* writes should be atomic from the node's point of view

Startup policy:

* if a valid persisted `forward_yaw` file exists, load it on startup
* use the loaded `forward_yaw` immediately as the authoritative committed value
* do not perform startup relearning when the file load succeeds
* disable further yaw learning for the remainder of that process lifetime
* if the file is missing, invalid, malformed, or unusable, fall back to fresh
  learning from scratch

If startup loading is enabled, the loaded `forward_yaw` must be treated as a
yaw-only flat-surface prior, not as a free 3D axis calibration.

---

## 6. Calibration and online learning

### 6.1 Startup convention

After boot:

* AHRS has already produced mounted `ahrs/imu`
* the speedometer has access to the gravity-leveled body surface through AHRS
  attitude
* if a valid persisted `forward_yaw` file exists, the speedometer starts with
  that loaded yaw as the authoritative committed value
* when startup load succeeds, the yaw learner is disabled and the loaded value
  remains fixed for that run
* if no valid file exists, the speedometer starts with no trusted learned yaw
  and may learn from scratch according to the runtime policy

### 6.2 Online learning model

The node may support online learning with checkpoints.

Maintain:

* `candidate_forward_yaw`
* `committed_forward_yaw`
* uncommitted learning evidence accumulated since the last checkpoint

This model applies only when startup did not load a valid persisted yaw. If
startup load succeeds, candidate/committed learning state is frozen and no new
learning evidence is accumulated during that run.

Policy:

* learning occurs only while motion is acceptable for straight-line yaw
  estimation on the gravity-leveled surface
* checkpoint commits are based on confidence and stability, not on ZUPT alone
* if turn detection fires, uncommitted learning evidence is discarded
* the last committed `forward_yaw` remains valid
* if startup load succeeds, online learning is disabled and checkpoint updates
  do not occur

### 6.3 Suitable learning evidence

Learning evidence comes from flat-surface motion that is consistent with the 1D
straight-line rail model.

At minimum, accepted learning periods should satisfy:

* valid `ahrs/imu`
* sufficient motion excitation on the gravity-leveled surface
* low model mismatch relative to the current candidate yaw
* no fused turn detection
* enough accepted evidence for stable candidate estimation

Useful evidence sources include:

* short-window motion direction on the gravity-leveled surface
* consistency of repeated forward/back motion along one yaw direction
* agreement of candidate updates across a recent evidence window

The speedometer should prefer learning from motion direction on the flat surface
rather than from raw instantaneous 3D acceleration direction in `base_link`.

### 6.4 Checkpoint commit rules

A checkpoint may be committed whenever the candidate yaw estimate is stable
enough.

Typical commit conditions include:

* minimum accepted sample count
* minimum excitation magnitude
* small recent change in `candidate_forward_yaw`
* sufficiently low orthogonal residual on the gravity-leveled surface
* no fused turn detection

ZUPT may be a convenient checkpoint moment when available, but it is not
required. Long forward/back/forward/back motion without ZUPT should still allow
online learning and checkpoint commits when fresh learning is enabled.

### 6.5 Learning stop / discard rules

Learning must stop when fused turn detection indicates the motion is not valid
for forward-yaw learning.

When that happens:

* discard uncommitted learning evidence since the last checkpoint
* keep the last committed `forward_yaw`
* continue runtime speed estimation if the committed forward yaw is valid

When startup load succeeds:

* no fresh yaw-learning updates are performed
* no candidate buffer is accumulated
* no checkpoint discard or commit logic is active for that run

---

## 7. Turn detection

### 7.1 Purpose

Turn detection is used to protect yaw learning. It is not, by itself, a reason
to invalidate the runtime scalar speed model.

The train may still have valid 1D forward speed while the body yaws through a
curve.

### 7.2 Fused turn detection

Turn detection should use fused evidence from both:

* angular velocity
* motion behavior on the gravity-leveled surface

This is preferred over a simple single-threshold rule.

Angular-velocity evidence helps detect turning onset. Flat-surface motion
evidence helps detect patterns inconsistent with straight-line yaw learning.

### 7.3 Runtime meaning

Turn detection affects:

* whether online learning is allowed
* whether uncommitted learning evidence is discarded

Turn detection does not automatically imply:

* invalid runtime speed
* automatic uncertainty inflation
* loss of the committed forward yaw

When startup load succeeds and learning is disabled, turn detection continues to
serve diagnostics and may still inform runtime behavior, but it does not drive
yaw-learning state transitions.

---

## 8. Runtime speed and angular-rate estimation

### 8.1 State and output quantities

The node estimates:

* scalar signed forward speed `forward_speed`
* scalar yaw angular velocity `yaw_rate`

The corresponding flat-surface linear velocity is:

* `v_B = forward_speed * forward_axis_flat`

The corresponding angular twist output is:

* `ω_G = [0, 0, yaw_rate]^T`

By policy:

* the node owns yaw angular velocity in the output twist
* the node does not estimate roll or pitch angular velocity
* the output angular x and y means are zero

### 8.2 Flat-surface projection

The runtime propagation should first map motion onto the gravity-leveled body
surface.

Then the forward component is computed by projecting onto the learned yaw axis:

* `a_forward = forward_axis_flat^T a_B_flat`

where:

* `a_B_flat` is the body acceleration projected into the `base_link` x-y plane

The speedometer should keep the propagation model deterministic and diagnosable.

### 8.3 Speed variance model

The node maintains an explicit scalar speed variance:

* `var_speed = var_forward_speed_mps2`

This is the uncertainty of the signed scalar speed along the learned forward
axis.

Propagation model:

* let `dt` be the accepted propagation interval in seconds
* let `q_a` be the process-noise variance of the projected forward acceleration
  in `(m/s^2)^2`

Then the speed variance grows as:

* `var_speed_k+ = var_speed_k + dt^2 * q_a`

This is the minimum required model. More elaborate models may add additional
terms, but they must preserve the interpretation that the node owns the scalar
speed uncertainty.

### 8.4 Drift control with ZUPT

ZUPT is used for scalar speed drift control.

When `zupt_flag` is true:

* apply a zero-speed correction toward `forward_speed = 0`
* use the `zupt` covariance as the uncertainty of that correction
* reduce speed uncertainty appropriately through the zero-speed update

This lets the node control drift without making ZUPT a requirement for online
yaw learning.

### 8.5 Yaw angular velocity model

The node publishes yaw angular velocity in the output twist.

The minimum runtime model is:

* `yaw_rate` is estimated from the mounted IMU angular velocity in
  `base_link`
* the mean output uses the `base_link` z component of angular velocity

That is:

* `yaw_rate = ω_B.z`

The node maintains an explicit scalar yaw-rate variance:

* `var_yaw_rate = var_yaw_rate_rads2`

This is the uncertainty of the output yaw angular velocity in `(rad/s)^2`.

By policy:

* the node owns the yaw-rate output covariance
* the node does not simply forward upstream driver covariance as the output
  model
* if a better internal model is unavailable, the node may use a conservative
  speedometer-owned yaw-rate variance tuned from observed IMU behavior

### 8.6 Runtime behavior during turns

Turns do not automatically invalidate runtime speed propagation.

Policy:

* continue speed estimation using the committed `forward_yaw`
* continue yaw-rate estimation from the current IMU stream
* do not inflate speed uncertainty merely because the body is yawing
* let uncertainty growth follow the actual propagation and correction model

If future evidence shows genuine model mismatch, that should be handled through
explicit model-mismatch logic rather than a blanket "turning means worse
uncertainty" rule.

### 8.7 Runtime behavior after startup load

When startup load succeeds:

* runtime speed propagation continues normally using the loaded
  `forward_yaw`
* yaw-rate estimation continues normally
* ZUPT drift control continues normally
* covariance propagation and publishing continue normally
* no new yaw-learning updates, checkpoint commits, or persistence rewrites occur
  during that run

---

## 9. Outputs

### 9.1 Primary output

Recommended primary topic:

* `ahrs/forward_twist`
* type: `geometry_msgs/TwistWithCovarianceStamped`

Preferred contract:

* `header.frame_id = "base_link"`
* `twist.twist.linear = forward_speed * forward_axis_flat`
* `twist.twist.angular = [0, 0, yaw_rate]`
* `covariance` is full and must not be diagonalized

### 9.2 Linear output semantics

The linear mean is always a vector, but it is constrained to the flat-surface
forward axis:

* `twist.twist.linear.x = forward_speed * cos(forward_yaw)`
* `twist.twist.linear.y = forward_speed * sin(forward_yaw)`
* `twist.twist.linear.z = 0`

This means the output vector may point in only two directions on the plane:

* along the learned forward axis
* or exactly opposite it

The magnitude of the linear mean is:

* `||twist.twist.linear|| = |forward_speed|`

### 9.3 Optional debug outputs

Useful optional debug/status outputs include:

* learned `forward_yaw`
* reconstructed `forward_axis_flat`
* candidate vs committed forward-yaw estimates
* learning state
* turn-detection state
* speed sigma for HUD use
* yaw sigma for calibration/debug use
* yaw-rate sigma for output diagnostics
* whether startup loaded a persisted yaw
* whether yaw learning is currently enabled

---

## 10. Covariance contract

### 10.1 General rule

The node must publish full covariance matrices and must never diagonalize by
convenience.

### 10.2 Upstream IMU covariance policy

The `ahrs/imu` covariance fields are an AHRS-owned output contract and should
not be treated as the authoritative uncertainty model for this node.

Current AHRS semantics are:

* roll/pitch covariance comes from gravity-observable attitude uncertainty
* yaw variance is handled separately by AHRS
* the full `ahrs/imu` covariance block is therefore not the speedometer's own
  uncertainty model

Therefore:

* do not use upstream IMU covariance as the authoritative speedometer
  uncertainty input
* do not simply rotate or forward those covariance blocks into the
  `forward_twist` output
* compute output covariance from the speedometer's own model

### 10.3 Separation of uncertainty states

The node owns three scalar uncertainty states:

* scalar speed uncertainty
* learned forward-yaw uncertainty
* yaw angular-velocity uncertainty

Define:

* `var_speed = var_forward_speed_mps2`
* `var_yaw = var_forward_yaw_rad2`
* `var_yaw_rate = var_yaw_rate_rads2`

Interpretation:

* `var_speed` is uncertainty along the learned forward axis
* `var_yaw` is uncertainty in the flat-surface direction itself
* `var_yaw_rate` is uncertainty of the published angular z component

### 10.4 Linear covariance model

Define:

* `s = forward_speed`
* `psi = forward_yaw`
* `u(psi) = [cos(psi), sin(psi), 0]^T`
* `n(psi) = [-sin(psi), cos(psi), 0]^T`

Then:

* `v_B = s * u(psi)`

The node models the linear velocity covariance as the sum of:

* speed contribution along the forward axis
* yaw contribution perpendicular to the forward axis on the flat surface

Specifically:

* `Sigma_speed = var_speed * u u^T`
* `Sigma_yaw = var_yaw * s^2 * n n^T`
* `Sigma_linear = Sigma_speed + Sigma_yaw`

Equivalent Jacobian form:

* `J_s = u(psi)`
* `J_psi = s * n(psi)`
* `Sigma_linear ≈ J_s var_speed J_s^T + J_psi var_yaw J_psi^T`

Geometric interpretation:

* speed uncertainty contributes along the learned forward axis
* yaw uncertainty contributes laterally on the flat surface
* when speed is small, yaw uncertainty matters less
* when speed is large, yaw uncertainty contributes more strongly because its
  effect scales with `s^2`

### 10.5 Linear z variance policy

The output linear z mean is zero by construction.

Policy for covariance:

* `Sigma_linear[2,2]` should ideally be zero
* all linear x-z and y-z covariances should ideally be zero
* the node should preserve this flat-surface constraint numerically

Numerical-stability rule:

* use exact zeros for the z row and z column of the linear covariance block
  whenever possible
* if implementation constraints require a floor, it must be extremely small and
  documented
* because downstream HUD logic may use a norm of covariance, artificial z
  variance should not be inflated

This is considered a public behavioral requirement, not a mere implementation
detail.

### 10.6 Angular covariance model

The node publishes angular mean:

* `[0, 0, yaw_rate]`

The angular covariance block should be:

* zero or extremely small for roll-rate and pitch-rate mean coupling owned by
  this node, because those means are not estimated here
* explicit and finite for yaw-rate variance

Minimum contract:

* `cov(ω_z, ω_z) = var_yaw_rate`
* cross-covariances with roll/pitch rates are zero unless explicitly modeled
* this node should publish only the angular covariance entries it actually
  owns, without inventing placeholder variances for unused axes

Preferred model:

* angular x and y variances are zero
* angular z variance is `var_yaw_rate`

### 10.7 Full 6x6 twist covariance

The published twist covariance should be arranged as:

* top-left `3 x 3`: `Sigma_linear`
* bottom-right `3 x 3`: angular covariance block for `[ω_x, ω_y, ω_z]`
* cross-covariances between linear and angular parts are zero unless explicitly
  modeled later

Minimum preferred structure:

```text
[ Sigma_linear      0 ]
[      0       Sigma_angular ]
```

with:

* `Sigma_linear = Sigma_speed + Sigma_yaw`
* `Sigma_angular = diag(0, 0, var_yaw_rate)`

No placeholder variances are needed for unowned angular dimensions in this
node's contract.

---

## 11. HUD contract

The HUD displays:

* `forward_speed`
* `± 1 sigma_speed`

That `1 sigma_speed` should be:

* `sqrt(var_speed)`

The HUD should not derive its uncertainty from an arbitrary norm of the full
twist covariance if that norm includes dimensions the HUD does not display.

In particular:

* yaw uncertainty belongs in the published twist covariance because it affects
  the vector direction
* yaw-rate uncertainty belongs in the published twist covariance because it
  affects angular z
* HUD speed uncertainty should come from scalar speed uncertainty only unless a
  separate direction-quality indicator is added

The user's suggestion that a covariance norm may be used in the HUD is not ideal
for this model, because it can be distorted by irrelevant dimensions such as
linear z or angular axes that are outside the HUD's scalar speed display. The
preferred HUD contract is explicitly
scalar:

* `forward_speed ± sqrt(var_speed)`

---

## 12. Time handling

Definitions:

* `t_meas_ns`: accepted measurement timestamp in integer nanoseconds
* `t_last_ns`: last accepted timestamp

Processing rule:

1. validate incoming data
2. update turn-detection state from IMU mean fields
3. map motion onto the gravity-leveled surface
4. if learning is allowed, update the candidate forward-yaw estimate
5. checkpoint candidate state when stability criteria are met
6. project flat-surface motion onto the committed forward-yaw axis
7. propagate scalar speed and speed variance
8. estimate yaw angular velocity and yaw-rate variance
9. if `zupt_flag` is true, apply zero-speed correction
10. publish `ahrs/forward_twist` and diagnostics

Ordering rule:

* out-of-order samples may be dropped deterministically
* no fixed-lag replay is required
* all acceptance, discard, and checkpoint rules must remain deterministic and
  diagnosable
* when startup load succeeds, runtime speed propagation and ZUPT drift control
  continue normally using the loaded `forward_yaw`
* when startup load succeeds, no new yaw-learning updates, checkpoint commits,
  or persistence rewrites occur during that run

---

## 13. Diagnostics

Recommended diagnostics fields:

* learning state
* candidate `forward_yaw`
* committed `forward_yaw`
* reconstructed `forward_axis_flat`
* accepted learning sample count
* rejected learning sample count
* discarded uncommitted sample count
* checkpoint commit count
* checkpoint discard count
* fused turn-detection state
* turn-detection trigger count
* current `forward_speed`
* current scalar speed variance
* current scalar speed sigma
* current forward-yaw uncertainty
* current forward-yaw sigma
* current `yaw_rate`
* current yaw-rate variance
* current yaw-rate sigma
* ZUPT correction count
* whether startup loaded a persisted `forward_yaw`
* loaded file path
* loaded file validity
* whether yaw learning is currently enabled
* why learning is disabled when startup load succeeds
* last accepted timestamp
* out-of-order drop count
* persistence write status

---

## 14. Parameters

Topics:

* `topics.imu` default `ahrs/imu`
* `topics.zupt` default `zupt`
* `topics.zupt_flag` default `zupt_flag`
* `topics.output_forward_twist` default `ahrs/forward_twist`

Frames:

* `base_frame_id` default `base_link`

Learning policy:

* minimum excitation magnitude
* minimum accepted sample count
* candidate stability threshold
* orthogonal residual threshold on the gravity-leveled surface
* fused turn-detection thresholds
* checkpoint commit thresholds
* learning-disable-on-load policy

Speed policy:

* forward-acceleration process-noise settings
* ZUPT correction enable/disable
* ZUPT measurement usage policy
* out-of-order sample handling
* covariance publication policy

Angular-rate policy:

* yaw-rate estimation policy
* yaw-rate variance policy
* angular covariance publication policy for the owned yaw-rate output only

Persistence policy:

* path for persisted `forward_yaw`
* write-on-checkpoint policy
* startup load policy
* fallback-to-fresh-learning policy when persisted files are invalid
* persistence rewrite disabled when startup load succeeds

---

## 15. Policy boundary

This node owns:

* forward-yaw learning on the gravity-leveled body surface
* startup loading of persisted `forward_yaw` when available
* disabling further yaw learning after a successful startup load
* online checkpointed estimation of `forward_yaw` when fresh learning is active
* signed scalar speed estimation along the learned flat-surface yaw axis
* yaw angular-velocity estimation for the output twist
* ZUPT-based speed drift control
* publication of `ahrs/forward_twist`
* output covariance computed from its own model
* persistence of `forward_yaw` for debugging and inspection

This node does not own:

* IMU mounting calibration
* AHRS attitude publication
* gravity estimation
* roll/pitch estimation
* global localization
* track-map estimation
* trust in generic upstream IMU covariance buckets as the node's uncertainty
  model
