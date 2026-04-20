# ZUPT

## Purpose

ZUPT is the stationary body-twist measurement used by OASIS to detect
stationary intervals from IMU data and publish an explicit zero body linear and
angular velocity measurement for downstream estimators.

The detector publishes:

- `zupt_flag` (`std_msgs/Bool`)
- `zupt` (`geometry_msgs/TwistWithCovarianceStamped`)

`zupt` is now a detector-owned stationary-twist contract. It is no longer the
older x-only linear placeholder contract.

---

## Measurement meaning

At each accepted timestamp, `zupt` means:

> the platform is believed to have zero body linear velocity and zero body
> angular velocity in the published frame, with the following detector-owned
> covariance

The published twist mean is:

- linear velocity: `[0, 0, 0]`
- angular velocity: `[0, 0, 0]`

---

## Covariance contract

The covariance is a real block-diagonal `6 x 6` stationary-twist covariance:

```text
[ Sigma_linear      0 ]
[      0       Sigma_angular ]
```

with:

- `Sigma_linear = var_linear * I_3`
- `Sigma_angular = var_angular * I_3`
- linear/angular cross terms equal to zero

Properties:

- symmetric
- positive semidefinite
- expressed in the same frame as the published twist
- linear block units of `m^2/s^2`
- angular block units of `(rad/s)^2`

This covariance represents stationary body-twist measurement uncertainty. It is
not a process model and not a placeholder artifact.

---

## Detector model

The detector consumes IMU samples and uses:

- angular-velocity norm
- linear-acceleration norm

The stationarity decision remains scalar and uses:

- gyro enter / exit thresholds
- accel enter / exit thresholds
- stationary / moving dwell timers
- optional entry smoothing

The detector owns two scalar covariance outputs:

- `var_linear`
- `var_angular`

Current first-pass policy:

- `var_linear` uses the linear stationary-twist model and inflates as gyro or
  accel evidence approaches the exit thresholds
- `var_angular` uses the angular stationary-twist model and inflates as gyro
  evidence approaches the exit threshold

While moving, both blocks switch to large configured variances so downstream
updates become negligible while the measurement presence remains explicit.

---

## Frame semantics

In full generality, a twist measurement may need to be re-expressed when moved
between frames.

OASIS intentionally defines the stationary-twist ZUPT measurement so
downstream consumers never need to perform that re-expression. This is a
permanent contract, not a temporary property of the current implementation.

The contract is permanently restricted to:

- zero linear mean
- zero angular mean
- isotropic `3 x 3` linear covariance block
- isotropic `3 x 3` angular covariance block
- zero linear/angular cross terms
- no future frame-structured extensions to this measurement

Under those restrictions, re-expression from the canonical `imu_link` message
into another frame is numerically unnecessary. The frame ID still matters
semantically, but downstream consumers may use the canonical `imu_link` ZUPT
message directly without frame-dependent rotation logic.

---

## ROS layout

For `geometry_msgs/TwistWithCovarianceStamped`, the intended layout is:

- top-left `3 x 3`: linear covariance block
- bottom-right `3 x 3`: angular covariance block
- all linear/angular cross terms zero by contract

This is the contract downstream consumers should assume.

---

## Relationship to AHRS

AHRS does not need to republish or rotate detector ZUPT just to change frames.

Downstream AHRS consumers may use the canonical `imu_link` ZUPT message
directly because the OASIS stationary-twist contract intentionally avoids any
frame-dependent numerical structure.

---

## Relationship to the forward speedometer

The forward speedometer consumes only the scalar zero-speed correction implied
by the stationary-twist measurement. It does not consume the full 6D twist
state directly.

In practice, that means:

- `zupt_flag` still acts as the primary Boolean stationarity signal
- the speedometer uses the relevant linear covariance entry for its scalar
  correction confidence
- the full stationary-twist contract remains available to other downstream
  consumers

---

## Parameters

Current detector parameters are:

- `linear_velocity_sigma_mps`
- `moving_linear_variance_mps2`
- `stationary_linear_variance_inflation`
- `angular_velocity_sigma_rads`
- `moving_angular_variance_rads2`
- `stationary_angular_variance_inflation`

These define the isotropic linear and angular covariance blocks for the
stationary-twist measurement.

---

## Summary

The permanent OASIS ZUPT contract is:

- zero body linear velocity in all three axes
- zero body angular velocity in all three axes
- detector-owned isotropic linear and angular covariance blocks
- zero linear/angular cross terms
- no frame-dependent numerical structure that downstream must re-express

That is the contract nearby code and docs should describe.
