# ZUPT

## Purpose

OASIS uses ZUPT as a detector-owned 6D stationary body-twist measurement.

The detector publishes:

- `zupt_flag` (`std_msgs/Bool`)
- `zupt` (`geometry_msgs/TwistWithCovarianceStamped`)

`zupt_flag` says whether the detector currently considers the platform
stationary.

`zupt` is always published alongside that state. Every published
`zupt_flag` state has a corresponding `zupt` message.

---

## Measurement meaning

`zupt` always carries the same zero-mean stationary-twist shape:

- linear velocity: `[0, 0, 0]`
- angular velocity: `[0, 0, 0]`

The covariance is always block diagonal:

```text
[ Sigma_linear      0 ]
[      0       Sigma_angular ]
```

with:

- `Sigma_linear = var_linear * I_3`
- `Sigma_angular = var_angular * I_3`
- all linear/angular cross terms equal to zero

Permanent OASIS contract:

- covariance never becomes anisotropic
- mean never becomes nonzero
- linear/angular cross-covariances are never introduced
- no frame-dependent covariance structure is introduced later

Downstream consumers should treat this as a stable contract.

---

## Detector model

The detector is intentionally simple and IMU-only.

It computes:

- gyro norm from the IMU angular-velocity field as provided upstream
- accel norm from the IMU linear-acceleration field as provided upstream

Stationary enter policy:

- require `gyro_norm <= gyro_enter_threshold_rads`
- require `accel_norm <= accel_enter_threshold_mps2`
- require both conditions to hold for `min_stationary_sec`

Stationary exit policy:

- trigger when `gyro_norm >= gyro_exit_threshold_rads` or
  `accel_norm >= accel_exit_threshold_mps2`
- require motion evidence to hold for `min_moving_sec`

This is a small hysteretic state machine with separate enter and exit
thresholds plus dwell timers. There is no detector-side smoothing, no
threshold-proximity covariance inflation, and no additional frame logic.

---

## Covariance policy

While stationary:

- publish one small isotropic linear variance
- publish one small isotropic angular variance

While moving:

- publish the same zero-mean stationary-twist shape
- switch to one very large isotropic linear variance
- switch to one very large isotropic angular variance

The moving-state message is intentional. It keeps the contract explicit
while making downstream updates negligible.

That means:

- `zupt_flag = true` pairs with a small-covariance stationary-twist
  measurement
- `zupt_flag = false` pairs with a huge-covariance stationary-twist
  measurement

---

## Frame semantics

OASIS intentionally keeps the ZUPT measurement frame-simple.

Because the mean is always zero, both covariance blocks are always
isotropic, and all cross terms remain zero, downstream consumers do not
need frame-rotation logic for ZUPT by contract.

Downstream code may consume the canonical detector `imu_link` ZUPT
directly.

---

## ROS layout

For `geometry_msgs/TwistWithCovarianceStamped`:

- top-left `3 x 3` block: isotropic linear covariance
- bottom-right `3 x 3` block: isotropic angular covariance
- all off-block terms: zero

This layout is permanent for OASIS ZUPT.

---

## Relationship to speedometers

AHRS and HUD speedometers currently consume only the scalar zero-speed
correction implied by the leading `3 x 3` linear covariance block.

In practice:

- `zupt_flag` remains the Boolean stationarity signal
- `zupt` remains the paired measurement
- small linear variance means an active zero-speed correction
- huge linear variance means a negligible zero-speed correction

No downstream frame rotation is required for this path.

---

## Parameters

Current detector parameters are:

- `gyro_enter_threshold_rads`
- `gyro_exit_threshold_rads`
- `accel_enter_threshold_mps2`
- `accel_exit_threshold_mps2`
- `min_stationary_sec`
- `min_moving_sec`
- `stationary_linear_velocity_sigma_mps`
- `stationary_angular_velocity_sigma_rads`
- `moving_linear_variance_mps2`
- `moving_angular_variance_rads2`

These fully define the current detector behavior and the stationary vs
moving covariance model.
