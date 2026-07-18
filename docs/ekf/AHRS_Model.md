# AHRS Model Specification

This document defines the mathematical and event model of the AHRS. The model
is independent of any particular implementation.

## Frames and rotation conventions

Frames are $W$ for world, $O$ for odom, $B$ for base, and $I$ for IMU.
Their default public names are `world`, `odom`, `base_link`, and
`imu_link`.

Every quaternion follows one convention: $q_{AB}$ rotates vectors from frame
$B$ into frame $A$. Therefore

$$
v_A=R(q_{AB})v_B,
$$

$$
q_{AC}=q_{AB}\otimes q_{BC},
$$

and

$$
q_{BA}=q_{AB}^*.
$$

ROS serializes quaternions as `(x, y, z, w)`; products below are Hamilton
products. Gravity points down, so level gravity in $B$ is approximately
$(0,0,-9.81)\,\mathrm{m/s^2}$.

## Accepted input samples

An IMU sample contains a timestamp, upstream ROS orientation, optional
orientation covariance, angular velocity and covariance, and acceleration and
covariance. The `imu` acceleration excludes gravity; the `imu_gravity`
acceleration includes gravity. Both are expressed in $I$.

An IMU sample is accepted when its frame is $I$, its quaternion is finite with
nonzero norm, its vectors are finite, and required covariance matrices are
finite with nonnegative diagonal entries. An orientation covariance whose
first entry is $-1$ is accepted as unknown. The BNO08X Rotation Vector follows
the device-to-reference convention and is copied unchanged into the upstream
ROS orientation. If $q_{\mathrm{ROS}}$ denotes that field, the canonical
IMU-to-world rotation is

$$
q_{WI}=\mathrm{normalize}(q_{\mathrm{ROS}}).
$$

No conjugation occurs at the AHRS input boundary. Its inverse
$q_{IW}=q_{WI}^*$ maps world-frame vectors into the IMU frame.

A gravity sample contains a timestamp, physical gravity vector $g_I$, and
optional $3\times3$ covariance $\Sigma_{gI}$. Its vector is accepted when
the frame is $I$, all components are finite, and
$\lVert g_I\rVert>10^{-9}$. An invalid covariance becomes unavailable; it
does not invalidate an otherwise acceptable gravity vector.

For `imu` and `gravity`, a sample is stale exactly when
$t<t_{last}$. Equal timestamps are accepted. Each stream has independent
timestamp history. The gravity-included IMU stream also drops timestamps less
than its own last accepted timestamp.

## Mounting and vector transformations

The fixed mounting rotation $R_{BI}$, represented by $q_{BI}$, maps
IMU-frame quantities into the base frame. Translation is zero. The rotation
comes from the boot calibration contract in `AHRS_Mounting.md`; no external
transform source participates.

The inverse mounting rotation $q_{IB}=q_{BI}^*$ maps base-frame vectors into
the IMU frame. The base-to-world attitude follows the frame chain
base-to-IMU-to-world:

$$
q_{WB}=q_{WI}\otimes q_{IB}
      =q_{WI}\otimes q_{BI}^*.
$$

The corresponding vector and covariance transformations are

$$
\omega_B=R_{BI}\omega_I,\qquad
a_B=R_{BI}a_I,
$$

$$
\Sigma_{\omega B}
=R_{BI}\Sigma_{\omega I}R_{BI}^{T},\qquad
\Sigma_{aB}
=R_{BI}\Sigma_{aI}R_{BI}^{T}.
$$

When upstream orientation covariance is known, the mounted
gravity-included output uses

$$
\Sigma_{qB}=R_{BI}\Sigma_{qI}R_{BI}^{T}.
$$

Gravity uses the same IMU-to-base mapping:

$$
g_B=R_{BI}g_I,\qquad
\Sigma_{gB}=R_{BI}\Sigma_{gI}R_{BI}^{T}.
$$

Full covariance matrices, including cross-axis terms, are retained.

## Gravity residual

The measured gravity direction in the base frame is

$$
u_m=\frac{g_B}{\lVert g_B\rVert}.
$$

Since $q_{WB}$ rotates base-frame vectors into the world frame, its inverse
$q_{BW}=q_{WB}^*$ rotates world-frame vectors into the base frame. The
predicted base-frame direction of world gravity is

$$
u_p=R(q_{BW})
\begin{bmatrix}
0\\
0\\
-1
\end{bmatrix}.
$$

The residual and its Euclidean diagnostic are

$$
r=u_m-u_p,\qquad d_r=\lVert r\rVert.
$$

`gravity_rejected` is true when a residual exists and $d_r>\tau_r$, where
$\tau_r$ defaults to $0.35$. Every processed `imu` event whose current
residual exceeds the threshold increments the rejection counter. Rejection is
diagnostic-only.

When gravity covariance is available, normalization uses the Jacobian

$$
J_u=\frac{I-u_m u_m^T}{\lVert g_B\rVert}.
$$

The normalized direction covariance is

$$
\Sigma_u=J_u\Sigma_{gB}J_u^T+10^{-9}I,
$$

and the Mahalanobis diagnostic is

$$
d_M=\sqrt{r^T\Sigma_u^{-1}r}.
$$

It is NaN when covariance inversion is unavailable and is never a gate.

## Attitude covariance

The session-yaw-zeroed attitude output derives roll and pitch uncertainty from
fresh gravity covariance. Gravity is fresh for an IMU timestamp $t_i$ when

$$
0\leq t_i-t_g\leq0.2\,\mathrm{s}.
$$

Let $g_B=(g_x,g_y,g_z)^T$, $h=\sqrt{g_y^2+g_z^2}$, and
$n^2=g_x^2+g_y^2+g_z^2$. The covariance propagation uses

$$
J_\phi=
\begin{bmatrix}
0 & g_z/h^2 & -g_y/h^2
\end{bmatrix},
$$

$$
J_\theta=
\begin{bmatrix}
h/n^2 &
-g_xg_y/(hn^2) &
-g_xg_z/(hn^2)
\end{bmatrix}.
$$

With

$$
J=
\begin{bmatrix}
J_\phi\\
J_\theta
\end{bmatrix},
$$

the roll/pitch covariance is $J\Sigma_{gB}J^T$. The $3\times3$
orientation covariance places that block in roll and pitch, sets yaw variance
to the fixed value $0\,\mathrm{rad^2}$, and sets roll/yaw and pitch/yaw
covariance to zero. Invalid, unavailable, or stale gravity covariance makes
orientation covariance unknown. Upstream orientation covariance does not
replace this policy on the session-yaw-zeroed output.

## Session yaw

For the first mounted `imu` attitude, let
$\psi_0=\mathrm{yaw}(q_{WB,0})$ and
$q_{OW}=q_z(-\psi_0)$. The session-relative base attitude is

$$
q_{OB}=\mathrm{normalize}(q_{OW}\otimes q_{WB}).
$$

The frame chain is base-to-world-to-odom, so $q_{OB}$ rotates base-frame
vectors into the odom/session frame. Its inverse is $q_{BO}=q_{OB}^*$. This
convention applies to the mounted gravity-removed IMU, odometry attitude, and
the quaternion serialized on `odom -> base_link`. It does not apply to
mounted gravity, the gravity-included IMU, or mounting TF.

## Event and publication semantics

Input streams are independent and are never paired or replayed. There is no
fixed-lag buffer and no clock-jump reset.

A gravity event validates and stores gravity, advances mounting calibration,
and publishes mounted gravity if mounting exists. An `imu_gravity` event
publishes its mounted sample if mounting exists. An `imu` event stores the
latest angular velocity for optional mounting stationarity evidence, evaluates
the current gravity residual when gravity exists, establishes or applies
session yaw, and publishes the primary IMU and odometry outputs when mounting
exists.

Measurement publications retain the triggering measurement timestamp.
Diagnostics and fixed TF refreshes use current time. Diagnostics and all
currently available TF edges are published after every input event and at
10 Hz.

The published TF state is:

- `world -> odom` is identity for the process lifetime.
- `base_link -> imu_link` contains $q_{BI}$ and zero translation. Its
  coefficients map IMU vectors into base coordinates, matching TF's
  child-to-parent transform direction for this edge.
- `odom -> base_link` contains $q_{OB}$ and zero translation after primary
  attitude is available. Its coefficients map base vectors into odom
  coordinates, matching TF's child-to-parent transform direction.

## Diagnostic state

Diagnostic state contains accepted, rejected, and stale IMU/gravity counters;
the gravity-residual rejection count; latest residual norm and Mahalanobis
distance; mounting availability; current gravity rejection; and bad-frame
state used to select the status code.

Status precedence is bad IMU frame, bad gravity frame, no accepted IMU, no
accepted gravity, mounting unavailable, then OK. Residual values are NaN
before a residual exists. No diagnostic state changes the numerical attitude
output.
