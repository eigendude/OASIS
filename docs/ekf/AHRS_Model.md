# AHRS Model Specification

This document defines the target structure for the current AHRS rewrite.

The current AHRS is a fused-IMU pipeline, not a raw multi-sensor optimizer.
Core logic should stay ROS-agnostic and operate on normalized `imu` and
`gravity` samples plus fixed mounting transforms.

This document is about implementation structure. Runtime behavior, published
contracts, and standalone TF policy belong in `AHRS.md`. Mounting-calibration
workflow belongs in `AHRS_Mounting.md`.

---

## 1. Directory layout target

Create the AHRS implementation under `oasis_control/oasis_control` with clear
separation between shared reusable localization utilities, AHRS-specific core
logic, and ROS wiring.

Suggested structure:

```text
oasis_control/oasis_control/localization/common/
  algebra/
    quat.py
    linalg.py
    covariance.py

  frames/
    mounting.py
    frame_policy.py

  data/
    imu_sample.py
    gravity_sample.py

  validation/
    imu_validation.py
    gravity_validation.py

  measurements/
    gravity_direction.py

oasis_control/oasis_control/localization/ahrs/
  data/
    ahrs_output.py
    diagnostics.py

  processing/
    attitude_mapper.py
    gravity_consistency.py
    output_adapter.py

  config/
    ahrs_config.py

oasis_control/oasis_control/nodes/ahrs/
  ahrs_node.py
  ros_messages.py
  ros_publishers.py
  ros_params.py

oasis_control/test/ahrs/
  common/
  processing/
```

Rules:

- core modules contain no ROS logging, QoS, or topic wiring
- node code only adapts ROS messages, parameters, TF, and diagnostics
- package `__init__.py` files stay empty

Why this split is better:

- `localization/common/` gives AHRS and EKF one shared home for low-level
  estimator-adjacent code that should not be reimplemented twice
- `common/data/` owns the canonical IMU and gravity packet types used by both
  components
- `common/frames/` owns frame-policy checks and fixed mounting helpers for
  `T_BI`
- `common/validation/` owns reusable IMU and gravity acceptance logic tied to
  the shared sensor contract
- `common/measurements/` owns gravity-direction residual math that both AHRS
  and EKF need, while leaving component-specific orchestration outside
- `localization/ahrs/` stays focused on AHRS outputs, AHRS-specific processing,
  and AHRS configuration instead of collecting duplicate low-level utilities

What should not move into `common/`:

- AHRS runtime publication policy
- AHRS standalone `world -> odom` TF ownership
- AHRS-specific output shaping and diagnostics contracts when they diverge from
  EKF needs

---

## 2. Shared conventions

Frames:

- `{W}` world
- `{O}` odom
- `{B}` base
- `{I}` IMU

Concrete public frame names:

- `world`
- `odom`
- `base_link`
- `imu_link`

Policy:

- `imu` samples are expected from `imu_link`
- `gravity` samples are expected from `imu_link`
- the fixed mounting transform is `T_BI` from `imu_link` to `base_link`
- mounting is expected to come from the boot-time calibration contract in
  `AHRS_Mounting.md`
- the default runtime path solves `T_BI` from boot gravity samples inside
  `ahrs_node` and publishes `base_link -> imu_link` as the fixed TF
- frame-policy enforcement belongs in `localization/common/frames/`

Quaternion convention:

- `q_WB` rotates world-frame vectors into `{B}` coordinates
- composition follows Hamilton product
- the fixed mounting quaternion is `q_BI`, which rotates vectors from
  `imu_link` into `base_link`
- publishing TF as parent=`base_link`, child=`imu_link` carries that same
  `q_BI`, not its inverse

Covariances:

- preserve full matrices
- never diagonalize incoming covariance by convenience
- publish transformed covariance via Jacobians or frame rotation as required
- rotate IMU and gravity covariance blocks when measurements are mapped through
  mounting transforms
- treat upstream IMU orientation covariance as driver-owned policy data; AHRS
  should preserve and rotate it rather than remapping SH-2 quality buckets
- treat driver-provided orientation covariance as the upstream contract rather
  than silently tightening or replacing it in AHRS

---

## 3. Core data types

The package boundary should read as:

- `localization/common/data/` holds the canonical IMU and gravity records
  shared by AHRS and EKF
- `localization/common/frames/` holds frame and mounting helpers that apply
  `T_BI`
- `localization/common/validation/` holds reusable IMU and gravity validation
- `localization/common/measurements/` holds reusable gravity-direction
  measurement math
- `localization/ahrs/data/` holds AHRS-specific outputs and diagnostic payloads
- `localization/ahrs/processing/` holds deterministic AHRS computation stages
  built on the shared common layer

### 3.1 `ImuSample`

Fields:

- timestamp in integer nanoseconds
- IMU frame id, expected to be `imu_link`
- canonicalized quaternion `q_WI`
- the current BNO086 driver publishes `q_IW`, so AHRS conjugates that packet
  to `q_WI` at the validation boundary
- optional orientation covariance
- angular velocity vector and covariance
- linear acceleration vector and covariance

### 3.2 `GravitySample`

Fields:

- timestamp in integer nanoseconds
- source frame id, expected to be `imu_link`
- physical gravity vector in `imu_link` that points down and is near
  `9.81 m/s^2` at rest
- full `3 x 3` covariance

This type is a first-class gravity measurement packet. It is not an "up"
vector, not a normalized unit vector, and not an acceleration used for
propagation.

### 3.3 `MountingTransform`

Fields:

- parent frame id
- child frame id
- quaternion `q_BI`

This is the fixed transform from `imu_link` to `base_link`.

### 3.4 `AhrsOutput`

Fields:

- timestamp
- base-frame quaternion `q_WB`
- base-frame angular velocity
- base-frame linear acceleration
- transformed covariance blocks
- gravity residual summary for diagnostics

---

## 4. Pipeline model

The AHRS remains a small event-driven system, but the implementation should be
split by responsibility instead of collecting all non-math code under one
generic `pipeline/` bucket.

The intended ownership is:

- shared sample validation, frame policy, mounting application helpers, and
  gravity-direction residual math live under `localization/common/`
- AHRS-specific attitude mapping, consistency policy, output shaping, and
  diagnostics remain under `localization/ahrs/`

### 4.1 IMU validation

The shared IMU validator in `localization/common/validation/` rejects samples
when:

- quaternion is non-finite
- quaternion norm is zero
- required covariance contains NaN or Inf
- IMU frame id does not match the expected `imu_link` policy

### 4.2 Gravity validation

The shared gravity validator in `localization/common/validation/` rejects
samples when:

- the vector is non-finite
- the vector norm is zero
- the covariance contains NaN or Inf
- the source frame does not match the expected `imu_link` policy

### 4.3 Mounting application

Given `q_WI` and `q_BI`:

- `q_WB = q_BI ⊗ q_WI`
- `ω_B = R_BI * ω_I`
- `a_B = R_BI * a_I`

Given `g_I`:

- `g_B = R_BI * g_I`

Covariances rotate the same way and remain full.

For orientation in particular, AHRS should rotate the `imu_link` covariance
into `base_link` under `T_BI` and then publish that rotated matrix unchanged.
If the incoming orientation covariance is poor or unknown, the runtime should
preserve that fact instead of inventing a new covariance model.

If a downstream tilt-only product is published, it should derive its own
roll/pitch covariance from gravity/down-direction observability rather than
reusing the full rotated AHRS attitude covariance.

This model assumes `T_BI` has already been solved from the boot gravity window
or provided by an explicit external calibration source before runtime
processing.
Boot-time mounting solve behavior belongs in `AHRS_Mounting.md`, not here.

The low-level `T_BI` application helper belongs in
`localization/common/frames/mounting.py`. AHRS-specific use of the mounted
result remains in `localization/ahrs/processing/`.

### 4.4 Gravity consistency

The gravity consistency stage compares the mounted attitude estimate against the
measured gravity direction in `base_link` under the shared convention
`WORLD_GRAVITY_DIRECTION = (0, 0, -1)`.

Responsibilities:

- evaluate the latest mounted gravity residual against explicit AHRS thresholds
- classify gravity as gated in or rejected for diagnostics/state reporting
- keep mounted AHRS outputs publishing when IMU and mounting are available
- avoid turning gravity consistency into a yaw correction or EKF-like update

- compute the predicted gravity direction from `q_WB`
- compare measured and predicted direction with full covariance
- use residual norm as the primary AHRS accept/reject gate
- keep Mahalanobis distance as a published diagnostic because normalized
  direction covariance can be overconfident for stationary checks
- expose a clear correction hook if gravity-based roll/pitch refinement is
  desired
- expose mounting consistency metrics for diagnostics or future online
  refinement

This stage must not invent yaw observability from gravity.

The reusable gravity-direction residual math should live in
`localization/common/measurements/gravity_direction.py`. AHRS keeps the
component-specific gating, reporting, and any refinement policy in
`localization/ahrs/processing/gravity_consistency.py`.

### 4.5 Node-facing integration responsibilities

The ROS/node integration layer is responsible for adapting accepted core AHRS
results into the standalone AHRS runtime products:

- in standalone AHRS mode, publish TF identity for `world -> odom`
- publish `odom -> base_link` from `q_WB`
- publish a mounted `sensor_msgs/Imu` equivalent in `base_link`
- publish diagnostics for IMU accept/reject counts, gravity accept/reject
  counts, and gravity residual status

This layer does not require a replay engine or separate measurement scheduler.

Suggested ROS-layer responsibilities:

- `ros_messages.py` maps `sensor_msgs/Imu` and
  `geometry_msgs/AccelWithCovarianceStamped` into
  `localization/common/data/` records and maps `AhrsOutput` from
  `localization/ahrs/data/` back into ROS messages
- `ros_publishers.py` owns topic publishers, diagnostics publication, and any
  optional odometry wrapper publication
- `ros_params.py` owns parameter declaration, validation, and frame-policy
  loading for the node

---

## 5. Testing targets

Unit tests should cover:

- quaternion normalization and rejection logic
- gravity vector validation
- frame rotation of vectors and covariance blocks
- mounting transform application
- gravity residual and gating behavior
- deterministic handling of out-of-order timestamps
- ROS conversion fidelity for `sensor_msgs/Imu` and
  `geometry_msgs/AccelWithCovarianceStamped`

---

## 6. Parameters

- input topic name for `imu`
- input topic name for `gravity`
- output topic names
- frame ids
- transform lookup policy
- out-of-order timestamp policy
- clock-jump reset threshold
- gravity residual gating policy
