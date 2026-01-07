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
AprilTag measurement model stubs for the EKF
"""

from typing import Optional

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
