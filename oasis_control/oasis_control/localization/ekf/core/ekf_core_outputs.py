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
Frame output helpers for the EKF core
"""

from __future__ import annotations

from oasis_control.localization.ekf.ekf_state import Pose3
from oasis_control.localization.ekf.ekf_types import EkfFrameOutputs
from oasis_control.localization.ekf.ekf_types import EkfFrameTransform
from oasis_control.localization.ekf.ekf_types import FrameId


class _EkfCoreOutputsMixin:
    def _frame_outputs(self) -> EkfFrameOutputs:
        t_odom_base: EkfFrameTransform = self._pose_to_transform(
            self._state.pose_ob,
            parent_frame="odom",
            child_frame="base",
        )
        t_world_odom: EkfFrameTransform = self._pose_to_transform(
            self._world_odom,
            parent_frame="world",
            child_frame="odom",
        )
        t_world_base_pose: Pose3 = self._compose_world_base()
        t_world_base: EkfFrameTransform = self._pose_to_transform(
            t_world_base_pose,
            parent_frame="world",
            child_frame="base",
        )
        return EkfFrameOutputs(
            t_odom_base=t_odom_base,
            t_world_odom=t_world_odom,
            t_world_base=t_world_base,
        )

    def _pose_to_transform(
        self, pose: Pose3, *, parent_frame: FrameId, child_frame: FrameId
    ) -> EkfFrameTransform:
        return EkfFrameTransform(
            parent_frame=parent_frame,
            child_frame=child_frame,
            translation_m=pose.translation_m.tolist(),
            rotation_wxyz=pose.rotation_wxyz.tolist(),
        )
