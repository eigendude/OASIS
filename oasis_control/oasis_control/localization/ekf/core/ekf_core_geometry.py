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
Geometry helpers for the EKF core
"""

from __future__ import annotations

import numpy as np

from oasis_control.localization.ekf.ekf_state import Pose3
from oasis_control.localization.ekf.se3 import pose_compose
from oasis_control.localization.ekf.se3 import quat_to_rpy


class _EkfCoreGeometryMixin:
    def _world_base_legacy_state(self) -> np.ndarray:
        world_base: Pose3 = self._compose_world_base()
        rpy: np.ndarray = quat_to_rpy(world_base.rotation_wxyz)
        zero_velocity: np.ndarray = np.zeros(3, dtype=float)
        return np.concatenate(
            (
                world_base.translation_m,
                zero_velocity,
                rpy,
            ),
            axis=0,
        )

    def _compose_world_base(self) -> Pose3:
        t_world_base: np.ndarray
        q_world_base: np.ndarray
        t_world_base, q_world_base = pose_compose(
            self._world_odom.translation_m,
            self._world_odom.rotation_wxyz,
            self._state.pose_ob.translation_m,
            self._state.pose_ob.rotation_wxyz,
        )
        return Pose3(translation_m=t_world_base, rotation_wxyz=q_world_base)

    def _lift_world_odom_jacobian(self, legacy_h: np.ndarray) -> np.ndarray:
        if legacy_h.shape[1] != 9:
            raise ValueError("Expected legacy Jacobian with 9 columns")
        lifted: np.ndarray = np.zeros((legacy_h.shape[0], 6), dtype=float)
        lifted[:, 0:3] = legacy_h[:, 0:3]
        lifted[:, 3:6] = legacy_h[:, 6:9]
        return lifted
