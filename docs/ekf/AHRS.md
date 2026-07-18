# AHRS Runtime Contract

The attitude and heading reference system (AHRS) adapts fused IMU attitude
streams to the robot base frame. It validates samples, solves the fixed
IMU-to-base tilt at startup, rotates vectors and covariances, establishes a
session-relative yaw origin, checks gravity consistency, and publishes
attitude products and TF.

The AHRS does not fuse raw inertial or magnetic observations, estimate bias or
translation, perform global localization, or correct `world -> odom`.

## Interfaces

The public topic names are fixed node names, not ROS parameters. System launch
files remap them under the hardware host, for example `imu` to
`<host>/imu` and `ahrs/imu` to `<host>/ahrs/imu`.

Inputs:

| Topic | Type | Meaning | QoS |
| --- | --- | --- | --- |
| `imu` | `sensor_msgs/Imu` | Fused attitude, angular velocity, and gravity-removed acceleration | reliable, keep last 512 |
| `imu_gravity` | `sensor_msgs/Imu` | Fused attitude, angular velocity, and gravity-included acceleration | reliable, keep last 256 |
| `gravity` | `geometry_msgs/AccelWithCovarianceStamped` | Physical gravity vector | reliable, keep last 256 |

Outputs:

| Topic | Type | Meaning | QoS |
| --- | --- | --- | --- |
| `ahrs/imu` | `sensor_msgs/Imu` | Mounted, session-yaw-zeroed gravity-removed IMU | reliable, keep last 512 |
| `ahrs/imu_gravity` | `sensor_msgs/Imu` | Mounted gravity-included IMU without session-yaw zeroing | reliable, keep last 512 |
| `ahrs/gravity` | `geometry_msgs/AccelWithCovarianceStamped` | Mounted gravity | reliable, keep last 256 |
| `ahrs/odom` | `nav_msgs/Odometry` | Session-yaw-zeroed attitude and angular velocity | best effort, keep last 10 |
| `ahrs/diag` | `oasis_msgs/AhrsStatus` | Structured runtime status | reliable, keep last 10 |

Both input IMU streams carry $q_{WI}$ in `orientation`. The mounted
gravity-included output carries $q_{WB}$. The session-yaw-zeroed IMU and
odometry outputs carry $q_{OB}$. These directions match ROS pose orientation
semantics: each quaternion maps the message's child/body frame into its
reference frame.

All QoS profiles are volatile and keep-last. Outputs are event-driven, except
that diagnostics and TF are also refreshed every 0.1 seconds.

## Frames and quaternions

The frame notation is $W$ for `world`, $O$ for `odom`, $B$ for
`base_link`, and $I$ for `imu_link`. The four names are parameters, but
these are their defaults.

ROS quaternions use `(x, y, z, w)`; equations use Hamilton products. The
universal convention is that $q_{AB}$ rotates vectors from frame $B$ into
frame $A$:

$$
v_A=R(q_{AB})v_B,\qquad
q_{AC}=q_{AB}\otimes q_{BC},\qquad
q_{BA}=q_{AB}^*.
$$

The upstream BNO08X Rotation Vector and the ROS orientation field contain
$q_{WI}$, which rotates IMU-frame vectors into the world frame. AHRS
normalizes this quaternion without conjugating it. The fixed mounting
quaternion $q_{BI}$ rotates IMU-frame vectors into the base frame, and its
inverse $q_{IB}=q_{BI}^*$ rotates base-frame vectors into the IMU frame.
The mounted base attitude is

$$
q_{WB}=q_{WI}\otimes q_{IB}
      =q_{WI}\otimes q_{BI}^*.
$$

The mounting transform has parent `base_link`, child `imu_link`, rotation
$q_{BI}$, and zero translation. It is solved internally from boot gravity;
there is no external mounting-transform source.

For any IMU-frame vector $v_I$ and covariance $\Sigma_I$,

$$
v_B=R_{BI}v_I,\qquad
\Sigma_B=R_{BI}\Sigma_I R_{BI}^{T}.
$$

This rule applies to angular velocity, linear acceleration, gravity, and their
available covariance blocks. Cross-axis terms are retained. Gravity points
down: a level stationary observation is approximately
$(0,0,-9.81)\,\mathrm{m/s^2}$.

## Session yaw

After mounting is available, the first accepted `imu` sample fixes the
session yaw. If its mounted yaw is $\psi_0$, AHRS stores the pure-yaw
world-to-odom rotation $q_{OW}=q_z(-\psi_0)$ and publishes

$$
q_{OB}=q_{OW}\otimes q_{WB}.
$$

$q_{OB}$ rotates base-frame vectors into the odom/session frame.

The offset remains fixed until restart. It affects only `ahrs/imu`,
`ahrs/odom`, and the `odom -> base_link` TF. It does not affect
`ahrs/imu_gravity`, `ahrs/gravity`, or `base_link -> imu_link`.
Consequently, gravity-included IMU output is mounted but not
session-yaw-zeroed.

## Gravity consistency and covariance

The latest mounted gravity direction is compared with the direction predicted
by the current mounted attitude. Because $q_{WB}$ maps base vectors into the
world frame, gravity prediction in the base frame uses
$q_{BW}=q_{WB}^*$:

$$
u_p=R(q_{BW})
\begin{bmatrix}
0\\
0\\
-1
\end{bmatrix}.
$$

A residual norm greater than
`gravity_residual_reject_threshold` sets `gravity_rejected` and increments
its counter. This decision is diagnostic-only: it does not suppress or alter
attitude publication.

When gravity covariance is usable, the same residual also has a Mahalanobis
distance computed from normalized-direction covariance. That distance is
diagnostic-only and is never a rejection gate.

The orientation covariance on `ahrs/imu` and `ahrs/odom` is not an
unconditional copy of upstream orientation covariance. Roll and pitch
covariance are propagated from the latest mounted gravity covariance through
the gravity-to-tilt Jacobian. Gravity must be no older than 0.2 seconds and
must not be newer than the IMU sample. Yaw variance follows the implemented
fixed policy of $0\,\mathrm{rad^2}$. If fresh usable gravity covariance is
unavailable, `ahrs/imu.orientation_covariance[0]` is `-1`; the odometry
orientation covariance remains unset.

`ahrs/imu_gravity` instead preserves the upstream orientation-covariance
unknown state and otherwise rotates the upstream covariance through mounting.

## Time, validation, and publication

Each input is processed independently. There is no pairing queue, fixed-lag
buffer, replay, or clock-jump reset policy. A timestamp is stale only when it
is less than the last accepted timestamp for that stream; equal timestamps are
accepted. Stale `imu` and `gravity` samples increment their respective
counters. Stale `imu_gravity` samples are silently dropped.

IMU samples require the configured IMU frame, a finite nonzero quaternion,
finite vectors, and finite covariance matrices with nonnegative diagonal
entries. The quaternion is normalized. The ROS unknown-orientation marker
(`orientation_covariance[0] == -1`) is accepted.

Gravity samples require the configured IMU frame and a finite nonzero vector.
An unusable gravity covariance is treated as unavailable rather than rejecting
the vector.

`ahrs/gravity` and `ahrs/imu_gravity` begin after mounting is solved.
`ahrs/imu`, `ahrs/odom`, and dynamic attitude TF begin with the first
accepted `imu` after mounting. Published measurement messages retain their
input timestamps. Diagnostic and fixed TF refreshes use current node time.

## TF contract

The AHRS broadcasts:

- `world -> odom`: identity, zero translation
- `base_link -> imu_link`: solved mounting, zero translation, once available
- `odom -> base_link`: session-relative $q_{OB}$, zero translation, once
  available

Every TF quaternion maps vectors from its child frame into its parent frame.

The AHRS owns these edges during standalone operation.

## Parameters

These are the complete supported AHRS parameters:

| Parameter | Default | Meaning |
| --- | ---: | --- |
| `base_frame_id` | `base_link` | Base frame |
| `imu_frame_id` | `imu_link` | Required input and mounting child frame |
| `odom_frame_id` | `odom` | Local attitude frame |
| `world_frame_id` | `world` | Identity parent of odom |
| `gravity_residual_reject_threshold` | `0.35` | Unit-direction residual diagnostic threshold |
| `mounting_calibration_duration_sec` | `2.0` | Minimum accepted-sample time span |
| `mounting_stationary_angular_speed_threshold_rads` | `0.35` | Maximum optional stationary angular speed in rad/s |
| `mounting_min_sample_count` | `10` | Minimum accepted gravity samples |

Topic names, gravity-covariance freshness, QoS, session-yaw policy, and timing
policies are not parameters.

## Diagnostics

`AhrsStatus.header.frame_id` is the configured world frame and its stamp is
current node time. `status` is one of OK, waiting for IMU, waiting for
gravity, bad IMU frame, bad gravity frame, or mounting unavailable. Bad-frame
states take precedence, followed by missing streams and mounting availability.

The message contains accepted, rejected, and stale counters for IMU and
gravity; the gravity-residual rejection count; the latest residual norm and
Mahalanobis distance (NaN before availability); `has_mounting`; and the
current `gravity_rejected` state. Gravity availability is derived from
`accepted_gravity_count`. There are no mounting status/result messages,
transform-lookup counters, rejection strings, or free-form status text.
