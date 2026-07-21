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

The canonical published frame is `imu_link`. Driver launch code supplies the
configured AHRS mounting child frame as the detector's `imu_frame_id`; a
standalone detector defaults to `imu_link`. The message retains the timestamp
of the triggering IMU sample.

This ZUPT is a detector-owned assertion that the rigid vehicle is stationary,
not a general twist estimate at the IMU origin. Its mathematical contract is:

```text
z = [0, 0, 0, 0, 0, 0]

P =
[ sigma_linear^2 * I3           0            ]
[          0           sigma_angular^2 * I3 ]
```

For any fixed mounting rotation `R`, define:

```text
A =
[ R  0 ]
[ 0  R ]

A * z = z
A * P * A^T = P
```

Therefore, consumers may apply this specific ZUPT without knowing the fixed
`imu_link`-to-`base_link` mounting rotation. They must reject a nonzero,
anisotropic, or cross-correlated ZUPT when they are not performing a real frame
transformation. Arbitrary twists and covariance matrices are not
frame-invariant.

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

## Stationarity measurement contract

Downstream consumers can interpret the detector output as follows:

- `zupt_flag` remains the Boolean stationarity signal
- `zupt` remains the paired measurement
- small linear variance means an active zero-speed correction
- huge linear variance means a negligible zero-speed correction

No downstream frame rotation is required for this exact measurement shape.

## QoS

`zupt` uses reliable, volatile, bounded QoS with a keep-last depth of 50. It is
not latched. `zupt_flag` uses reliable, transient-local QoS with keep-last depth
1 so a late subscriber receives the current stationarity state.

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
- `imu_frame_id` (default `imu_link`)

These fully define the current detector behavior and the stationary vs
moving covariance model.
