################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""
AprilTag measurement model helpers for the EKF
"""

from typing import Optional

import numpy as np

from oasis_control.localization.ekf.ekf_types import AprilTagDetection
from oasis_control.localization.ekf.ekf_types import CameraInfoData


class AprilTagMeasurementModel:
    """
    Placeholder AprilTag measurement model
    """

    def __init__(self) -> None:
        self._camera_info: Optional[CameraInfoData] = None

    def update(self) -> None:
        """
        Apply an AprilTag measurement update
        """

        # TODO: Implement reprojection residuals and Jacobians
        return None

    def set_camera_info(self, camera_info: CameraInfoData) -> None:
        """
        Cache camera intrinsics for future reprojection work
        """

        self._camera_info = camera_info
        # TODO: Use intrinsics for corner reprojection

    def pose_measurement(self, detection: AprilTagDetection) -> Optional[np.ndarray]:
        if detection.pose_world_xyz_yaw is None:
            return None
        if len(detection.pose_world_xyz_yaw) != 4:
            return None
        return np.asarray(detection.pose_world_xyz_yaw, dtype=float)

    def linearize_pose(self, state: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        z_hat: np.ndarray = np.asarray(
            [state[0], state[1], state[2], state[8]], dtype=float
        )
        h: np.ndarray = np.zeros((4, 9), dtype=float)
        h[0, 0] = 1.0
        h[1, 1] = 1.0
        h[2, 2] = 1.0
        h[3, 8] = 1.0
        return z_hat, h
