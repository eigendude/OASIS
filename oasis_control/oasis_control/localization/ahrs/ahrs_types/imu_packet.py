################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""IMU measurement packet definition for the AHRS core.

Responsibility:
    Document the IMU measurement fields required by the AHRS filter without
    introducing ROS message types.

Inputs/outputs:
    - Inputs: raw gyro and accel measurements with covariances.
    - Outputs: structured packet used by ImuModel and AhrsEkf.

Dependencies:
    - Consumed by ImuModel and ReplayEngine.

Determinism:
    IMU packets are immutable once created; fields must not change during
    replay.
    IMU packets are created only from an ExactTime-synchronized
    (imu_raw, imu_calibration) pair.
"""


class ImuPacket:
    """IMU measurement packet for gyro and accelerometer updates.

    Purpose:
        Provide a single data container for gyro and accelerometer samples
        with covariances and frame identifiers.

    Public API (to be implemented):
        - validate()
        - as_dict()

    Data contract:
        Required fields:
        - t_meas_ns: measurement timestamp in int nanoseconds since an
          arbitrary epoch.
        - frame_id: sensor frame identifier for {I}.
        - z_omega: gyro measurement vector (3,).
        - R_omega: gyro covariance matrix (3, 3).
        - z_accel: accel measurement vector (3,).
        - R_accel: accel covariance matrix (3, 3).
        - calibration_prior: calibration payload paired with the measurement,
          including (b_a, A_a, b_g) and the associated full covariance.
        - calibration_meta: metadata for calibration source and validity.

    Frames and units:
        - z_omega in rad/s, frame {I}.
        - z_accel in m/s^2, frame {I} (specific force, a - g).
        - Covariances use squared units.

    Determinism and edge cases:
        - All timestamps are int nanoseconds since an arbitrary epoch.
          The epoch is irrelevant because only differences and exact
          equality are used.
        - t_meas_ns equality is exact for buffer keying.
        - No epsilon merging or rounding-based matching is permitted.
        - Packets must contain both z_omega and z_accel with covariances.
        - Packets must be dropped if an ExactTime-synchronized calibration
          payload is missing for the same t_meas_ns.
        - validate() rejects missing z_omega/z_accel or missing covariances.

    Equations:
        - No equations; this is a data container.

    Numerical stability notes:
        - Ensure R_omega and R_accel are SPD.

    Suggested unit tests:
        - validate rejects wrong shapes.
        - validate rejects non-SPD covariances.
    """

    pass
