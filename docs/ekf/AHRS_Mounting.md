# AHRS Mounting Calibration Contract

Mounting calibration determines the fixed tilt rotation from the IMU frame
$I$, default `imu_link`, into the base frame $B$, default
`base_link`. It runs automatically at startup as part of the AHRS runtime.

Quaternion subscripts follow the universal convention: $q_{AB}$ rotates
vectors from frame $B$ into frame $A$. Therefore
$v_A=R(q_{AB})v_B$, $q_{AC}=q_{AB}\otimes q_{BC}$, and
$q_{BA}=q_{AB}^*$.

## Observations

Physical gravity is the required observation. Each accepted observation is a
finite, nonzero `geometry_msgs/AccelWithCovarianceStamped` gravity vector in
$I$. The vector points down and is normalized before accumulation.
Gravity covariance is not used to weight, accept, or solve the mounting.

Angular velocity from the latest accepted gravity-removed
`sensor_msgs/Imu` sample is optional stationary evidence. When it is present,
finite, and its norm exceeds the configured threshold, the associated gravity
sample is rejected from calibration. If angular velocity is absent or
nonfinite, gravity can still be accepted.

IMU orientation and linear acceleration are not mounting-solver inputs. The
gravity-included IMU stream is not a mounting input.

## Acceptance and accumulation

For each accepted gravity sample $g_{I,k}$, calibration accumulates the unit
direction

$$
u_{I,k}=\frac{g_{I,k}}{\lVert g_{I,k}\rVert}.
$$

The calibration start time is the timestamp of the first accepted sample.
Rejected high-angular-rate samples neither contribute nor reset the start
time, sum, count, or any previously accumulated calibration state.

The solve becomes eligible only when both conditions hold:

$$
t_{last}-t_{first}\geq T_{min}
$$

and

$$
N\geq N_{min},
$$

where the time span and count include accepted samples only. The mean
direction used by the solve is

$$
\bar{u}_I=
\frac{\sum_{k=1}^{N}u_{I,k}}
     {\left\lVert\sum_{k=1}^{N}u_{I,k}\right\rVert}.
$$

Zero or nonfinite gravity is rejected before accumulation. Configuration
duration and angular-rate threshold values below zero are clamped to zero;
minimum count values below one are clamped to one.

## Observable solution

The solution $R_{BI}$ maps IMU-frame vectors into the base frame:

$$
v_B=R_{BI}v_I.
$$

For $\bar{u}_I=(u_x,u_y,u_z)^T$, the solved mounting angles are

$$
\phi=\mathrm{atan2}(-u_y,-u_z),
$$

$$
u_z'=\sin(\phi)u_y+\cos(\phi)u_z,
$$

$$
\theta=\mathrm{atan2}(u_x,-u_z').
$$

The mounting rotation is formed from roll $\phi$, pitch $\theta$, and yaw
zero. Its quaternion $q_{BI}$ and matrix $R_{BI}$ rotate IMU-frame vectors
into the base frame. They map the averaged gravity direction to the level base
direction $(0,0,-1)$.

Gravity makes roll and pitch observable in a stationary pose. It provides no
mounting-yaw observation, so mounting yaw is fixed to zero. There is no
magnetic, visual, external-transform, or other heading source. Translation is
also unobserved and fixed to zero.

## Runtime ownership and TF

The AHRS owns the calibration process and its result. Once solved, the result
is immutable for the remainder of that process lifetime and is used for all
mounted AHRS outputs.

The result exists only in memory. There is no persistence, loading, service,
separate calibration process, or external mounting source. Restarting the AHRS
starts a new calibration.

The published transform is

$$
T_{BI}=(R_{BI},0),
$$

with parent `base_link` and child `imu_link`. Its quaternion $q_{BI}$
rotates child-frame vectors into the parent frame. Translation is exactly
zero.

## Parameters

| Parameter | Default | Contract |
| --- | ---: | --- |
| `base_frame_id` | `base_link` | TF parent and mounted output frame |
| `imu_frame_id` | `imu_link` | Required observation frame and TF child |
| `mounting_calibration_duration_sec` | `2.0` | Minimum accepted-sample span in seconds |
| `mounting_stationary_angular_speed_threshold_rads` | `0.35` | Maximum optional angular-speed evidence in rad/s |
| `mounting_min_sample_count` | `10` | Minimum accepted gravity count |

Topics are fixed runtime interface names and are remapped by system launch;
they are not parameters.

## Availability and diagnostics

Mounting is unavailable until the duration and count requirements are both
satisfied and the averaged direction yields a finite solution. Primary AHRS
outputs that require mounting do not publish before that point.

The compact AHRS diagnostic reports `has_mounting` and the overall
`STATUS_MOUNTING_UNAVAILABLE` state. Accepted and rejected gravity counters
describe input validation, but there are no calibration-window counters,
high-rate rejection counters, persistence diagnostics, lookup diagnostics, or
separate mounting status/result messages.
