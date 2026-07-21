# AHRS Forward Speedometer Design Document

This document defines the AHRS-based forward-speed contract for the current
rail-vehicle motion model.

The system uses a BNO IMU that provides fused roll, pitch, and yaw. The AHRS
mounts that attitude into `base_link` and establishes a boot reference in which
the vehicle attitude is roll `0`, pitch `0`, and yaw `0`. The BNO attitude is
authoritative after that initialization.

## 1. Goals

- Estimate signed one-dimensional vehicle speed.
- Consume mounted BNO data from `ahrs/imu`.
- Treat AHRS roll, pitch, and yaw as observed inputs.
- Use the boot-calibrated attitude as the zero reference.
- Use `base_link +x` as the vehicle-forward direction.
- Use ZUPT observations to control integrated speed drift.
- Publish forward linear velocity and all mounted angular-velocity components.
- Publish covariance owned by the speedometer.

## 2. Non-goals

- No attitude or heading estimation.
- No forward-axis estimation.
- No IMU mounting calibration.
- No lateral or vertical velocity estimation.
- No global position estimation.
- No track-map or curvature estimation.
- No separate compass model.
- No separate gravity-topic dependency.

## 3. Frames and boot reference

Frames:

- `world`: AHRS world/reference frame.
- `base_link`: mounted vehicle body frame.

The fixed longitudinal vehicle axis is:

```text
forward_axis_B = [1, 0, 0]
```

The BNO supplies fused three-axis attitude. During boot calibration, the current
mounted attitude becomes the public zero reference:

```text
roll  = 0
pitch = 0
yaw   = 0
```

After boot, roll, pitch, and yaw describe rotation away from that reference.
Yaw is directly observed by the BNO fusion solution; it is not inferred from
vehicle motion. In this document, "absolute yaw" means yaw is fully observed by
the sensor rather than left unobservable. Because the public attitude is zeroed
at boot, public yaw remains relative to the boot reference unless a separate
Earth-referenced convention is explicitly configured.

The speedometer must not add another attitude zero, heading offset, or mounting
correction.

## 4. ROS inputs

### 4.1 AHRS IMU

- Topic: `ahrs/imu`
- Type: `sensor_msgs/Imu`
- Required frame: `base_link`

Used fields:

- `header.stamp`
- `orientation`
- `orientation_covariance`
- `angular_velocity`
- `angular_velocity_covariance`
- `linear_acceleration`
- `linear_acceleration_covariance`

Input semantics:

- `orientation` is the mounted, boot-zeroed BNO attitude.
- `orientation_covariance` describes roll, pitch, and yaw uncertainty for that
  mounted, boot-zeroed attitude.
- `angular_velocity` is expressed in `base_link`.
- `angular_velocity_covariance` describes uncertainty in the mounted
  `base_link` angular velocity.
- `linear_acceleration` is expressed in `base_link` and excludes gravity.
- `linear_acceleration_covariance` describes uncertainty in the mounted,
  gravity-removed `base_link` acceleration.
- Covariances are row-major `3 x 3` matrices. Cross-axis terms are retained.
- A covariance whose first element is `-1` is treated as unavailable, following
  the `sensor_msgs/Imu` convention; it does not invalidate an otherwise valid
  measurement.
- Invalid, non-finite, wrong-frame, or out-of-order measurements are rejected.
- An available covariance is rejected if it contains non-finite values or a
  negative diagonal entry.

The speedometer validates orientation and its covariance but does not rotate
acceleration, compensate attitude, modify orientation, or re-estimate it. Valid
input covariances inform the speedometer's process and measurement uncertainty,
but they are not copied unchanged into the output twist covariance.

### 4.2 ZUPT

- Topic: `zupt`
- Type: `geometry_msgs/TwistWithCovarianceStamped`
- Canonical publisher frame: `imu_link`
- Topic: `zupt_flag`
- Type: `std_msgs/Bool`

`zupt_flag` indicates that the vehicle is stationary. The corresponding `zupt`
sample supplies the stationary velocity observation and its covariance.

The speedometer intentionally does not require or inspect a particular ZUPT
frame ID. No mounting rotation is needed for this fixed zero-mean covariance:
its linear and angular blocks are independently isotropic and all cross terms
are zero. This is a narrow ZUPT exception, not general covariance frame
conversion.

Before applying the correction, the speedometer validates that all six mean
components are finite and zero, both covariance blocks are positive isotropic
matrices, all within-block off-diagonal and linear-angular cross terms are
zero, and the complete covariance is finite, symmetric, and positive
semidefinite. A malformed ZUPT is rejected regardless of its frame ID.

ZUPT corrects the speed and mounted angular-rate substate and their complete
covariance. It does not alter attitude or the boot reference.

## 5. Motion model

The rail-vehicle velocity is constrained to the longitudinal axis of
`base_link`:

```text
v_B = [forward_speed, 0, 0]
```

Positive speed is motion along `base_link +x`; negative speed is motion along
`base_link -x`.

The longitudinal acceleration used for propagation is the already-mounted,
gravity-free acceleration along `base_link +x`:

```text
a_forward = linear_acceleration.x
```

The speedometer does not rotate the acceleration or estimate an axis.

For an accepted interval `dt`:

```text
forward_speed_next = forward_speed + a_forward * dt
```

Implementations may use filtering and bias handling, but those mechanisms must
not introduce a second heading or attitude state.

## 6. Drift control

When `zupt_flag` is true and a valid `zupt` sample is available:

- Treat the forward-speed observation as zero.
- Use the forward linear-velocity variance from the `zupt` covariance as the
  correction uncertainty.
- Require that variance to be finite and nonnegative.
- Correct `forward_speed` toward zero according to the measurement uncertainty.
- Reduce `var_speed` according to the same correction model.
- Reject the correction if the ZUPT measurement or required covariance is
  invalid.
- Do not change the BNO attitude or boot reference.
- Use the complete `6 x 6` `zupt` covariance in the zero-twist correction.
- Propagate the complete posterior covariance to `ahrs/forward_twist`.
- Retain cross-axis and linear-angular covariance terms.

No stored speedometer calibration is required for normal operation.

## 7. Angular velocity and combined covariance

The output angular velocity comes directly from the mounted IMU:

```text
angular = angular_velocity
````

All three components are published:

```text
angular = [roll_rate, pitch_rate, yaw_rate]
```

The complete row-major `3 x 3` `angular_velocity_covariance` is propagated into
the angular block of the output twist covariance. Cross-axis covariance terms
are retained.

The output uses the standard twist ordering:

```text
[linear.x, linear.y, linear.z, angular.x, angular.y, angular.z]
```

Before a ZUPT correction, the combined covariance contains:

```text
[ linear covariance          linear-angular covariance ]
[ angular-linear covariance  angular covariance        ]
```

The angular block is initialized from the complete mounted IMU
`angular_velocity_covariance`. The linear block is propagated from the speed
state and mounted linear-acceleration covariance. Cross-covariance terms are
retained whenever they are available or produced by the estimator.

When a valid ZUPT observation is applied, its complete `6 x 6` twist covariance
is used in the correction. The resulting complete posterior covariance is
published on `ahrs/forward_twist`; it must not be reduced to selected diagonal
entries or reconstructed as a diagonal matrix.

Turning does not change the configured forward axis and does not invalidate the
attitude. The BNO continues to provide fused attitude and complete mounted
angular velocity while the speedometer estimates longitudinal speed.

## 8. Output

Primary output:

- Topic: `ahrs/forward_twist`
- Type: `geometry_msgs/TwistWithCovarianceStamped`
- Frame: `base_link`

Mean values:

```text
linear  = [forward_speed, 0, 0]
angular = [roll_rate, pitch_rate, yaw_rate]
```

The output retains the triggering measurement timestamp.

Useful diagnostics include:

- current forward speed
- current mounted angular velocity
- scalar speed variance and standard deviation
- angular-rate covariance and standard deviations
- ZUPT correction count
- rejected and out-of-order sample counts
- boot calibration readiness
- BNO calibration/readiness state, when available
- last accepted timestamp

## 9. Covariance

The speedometer owns the complete output twist covariance. It combines
uncertainty from acceleration propagation, mounted angular velocity, and ZUPT
corrections. It must not blindly copy an input covariance or reduce the result
to a diagonal matrix.

The twist state uses the standard ordering:

```text
[linear.x, linear.y, linear.z, angular.x, angular.y, angular.z]
````

Let `P_twist` be its complete `6 x 6` covariance.

For an accepted propagation interval `dt`, the linear-velocity covariance is
propagated using the mounted linear-acceleration covariance:

```text
P_linear_next = P_linear + dt^2 * acceleration_covariance
```

This form assumes `acceleration_covariance` contains per-sample acceleration
variance in `(m/s^2)^2`. If a configured value is instead a continuous-time
noise density, its discretization must be documented separately.

The complete mounted `3 x 3` angular-velocity covariance is used for the angular
block of `P_twist`. Cross-axis terms are retained.

When a valid ZUPT is active, its complete `6 x 6` covariance is used in the
zero-twist correction. The complete posterior covariance is then published on
`ahrs/forward_twist`, including any linear, angular, and linear-angular
cross-covariance produced by the estimator.

The covariance must:

* use the standard ROS twist ordering
* be finite
* have nonnegative diagonal entries
* retain modeled off-diagonal entries
* remain symmetric
* not be replaced by a diagonal approximation

The HUD displays signed forward speed as:

```text
forward_speed = forward_twist.twist.linear.x
```

Its variance and standard deviation are therefore:

```text
var_forward_speed = forward_twist.twist.covariance[0]
sigma_forward_speed = sqrt(var_forward_speed)
```

This remains correct when the published matrix contains off-diagonal terms:
`covariance[0]` is the marginal variance of `linear.x`. The HUD should not use
the norm, trace, or sum of the complete covariance matrix for its scalar speed
uncertainty.

## 10. Processing order

For each accepted IMU sample:

1. Validate the frame, timestamp, quaternion, vectors, and required covariance.
2. Use the AHRS orientation as supplied.
3. Obtain longitudinal acceleration along `base_link +x`.
4. Propagate forward speed and scalar speed variance.
5. Obtain all mounted angular-rate components from `angular_velocity`.
6. Apply a zero-speed correction when a valid ZUPT is active.
7. Publish `ahrs/forward_twist` and diagnostics.

Streams remain independent. Out-of-order input may be dropped deterministically;
fixed-lag replay is not required.

## 11. Parameters

Topics:

- `imu`, maps to `ahrs/imu`
- `zupt`, maps to `zupt`
- `zupt_flag`, maps to `zupt_flag`
- `forward_twist`, maps to `ahrs/forward_twist`

Do not add ROS parameters for topics. The mapping is done in the launchfile.

Frames:

- `base_frame_id`, default `base_link`

Speed policy:

- longitudinal acceleration process noise
- speed-filter and bias settings, if implemented
- ZUPT correction policy
- out-of-order sample policy
- covariance publication policy

Angular-rate policy:

- unavailable angular covariance fallback
- complete angular covariance publication policy

## 12. Ownership boundary

The AHRS owns:

- BNO initialization and calibration
- IMU mounting
- fused roll, pitch, and yaw
- boot zeroing to roll `0`, pitch `0`, and yaw `0`
- mounted acceleration and angular velocity

The forward speedometer owns:

- signed scalar speed along `base_link +x`
- speed drift control using ZUPT
- complete mounted angular-rate mapping for the forward-twist output
- forward-twist covariance
- `ahrs/forward_twist` publication

The forward speedometer does not create, refine, or replace any AHRS attitude
or heading value.
