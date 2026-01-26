################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for mounting calibration state containers."""

from __future__ import annotations

from dataclasses import FrozenInstanceError

import numpy as np
import pytest

from oasis_control.localization.mounting.state.mounting_state import KeyframeAttitude
from oasis_control.localization.mounting.state.mounting_state import MountEstimate
from oasis_control.localization.mounting.state.mounting_state import MountingState
from oasis_control.localization.mounting.state.mounting_state import MountingStateError


def test_mounting_state_default_validates() -> None:
    """Ensure the default state validates."""
    state: MountingState = MountingState.default()
    state.validate()


def test_mount_estimate_normalizes_quaternion() -> None:
    """Ensure quaternions are normalized on construction."""
    mount: MountEstimate = MountEstimate(
        q_BI_wxyz=np.array([2.0, 0.0, 0.0, 0.0], dtype=np.float64),
        q_BM_wxyz=np.array([0.0, 0.0, 0.0, -3.0], dtype=np.float64),
        p_BI_m=np.zeros(3, dtype=np.float64),
        p_BM_m=np.zeros(3, dtype=np.float64),
    )
    assert np.isclose(np.linalg.norm(mount.q_BI_wxyz), 1.0)
    assert np.isclose(np.linalg.norm(mount.q_BM_wxyz), 1.0)


def test_keyframe_ordering_and_uniqueness() -> None:
    """Ensure keyframes are sorted and unique."""
    kf_1: KeyframeAttitude = KeyframeAttitude(
        keyframe_id=2,
        q_WB_wxyz=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
    )
    kf_2: KeyframeAttitude = KeyframeAttitude(
        keyframe_id=1,
        q_WB_wxyz=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
    )
    state: MountingState = MountingState.default().replace(keyframes=(kf_1, kf_2))
    assert state.keyframe_ids() == (1, 2)

    with pytest.raises(MountingStateError):
        MountingState.default().replace(keyframes=(kf_2, kf_2))


def test_with_updated_keyframe_inserts_and_replaces() -> None:
    """Ensure keyframe insertion and replacement are deterministic."""
    state: MountingState = MountingState.default()
    kf_1: KeyframeAttitude = KeyframeAttitude(
        keyframe_id=5,
        q_WB_wxyz=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
    )
    state = state.with_updated_keyframe(kf_1)
    assert state.keyframe_ids() == (5,)

    kf_1_updated: KeyframeAttitude = KeyframeAttitude(
        keyframe_id=5,
        q_WB_wxyz=np.array([0.0, 1.0, 0.0, 0.0], dtype=np.float64),
    )
    state = state.with_updated_keyframe(kf_1_updated)
    assert state.get_keyframe(5).q_WB_wxyz[1] == pytest.approx(1.0)


def test_mounting_state_replace_and_frozen() -> None:
    """Ensure replace works and state remains frozen."""
    state: MountingState = MountingState.default()
    updated: MountingState = state.replace(anchored=True)
    assert updated.anchored is True
    with pytest.raises(FrozenInstanceError):
        state.anchored = True  # type: ignore[misc]
