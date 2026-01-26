################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for state mapping block layout."""

from __future__ import annotations

import numpy as np

from oasis_control.localization.mounting.state.mounting_state import KeyframeAttitude
from oasis_control.localization.mounting.state.mounting_state import MountingState
from oasis_control.localization.mounting.state.state_mapping import BLOCK_NAME_A_A
from oasis_control.localization.mounting.state.state_mapping import BLOCK_NAME_B_A
from oasis_control.localization.mounting.state.state_mapping import BLOCK_NAME_B_G
from oasis_control.localization.mounting.state.state_mapping import BLOCK_NAME_G_W
from oasis_control.localization.mounting.state.state_mapping import BLOCK_NAME_P_BI
from oasis_control.localization.mounting.state.state_mapping import BLOCK_NAME_P_BM
from oasis_control.localization.mounting.state.state_mapping import BLOCK_NAME_R_BI
from oasis_control.localization.mounting.state.state_mapping import BLOCK_NAME_R_BM
from oasis_control.localization.mounting.state.state_mapping import StateMapping


def test_state_mapping_default_blocks() -> None:
    """Ensure default state mapping contains expected blocks."""
    state: MountingState = MountingState.default()
    mapping: StateMapping = StateMapping.from_state(
        state,
        include_translation_vars=False,
    )
    assert mapping.has(BLOCK_NAME_R_BI)
    assert mapping.has(BLOCK_NAME_R_BM)
    assert mapping.has(BLOCK_NAME_G_W)
    assert mapping.has(BLOCK_NAME_B_A)
    assert mapping.has(BLOCK_NAME_A_A)
    assert mapping.has(BLOCK_NAME_B_G)
    assert not mapping.has(BLOCK_NAME_P_BI)
    assert not mapping.has(BLOCK_NAME_P_BM)
    blocks = mapping.blocks()
    assert mapping.dim() == sum(block.dim for block in blocks)
    for index, block in enumerate(blocks):
        if index == 0:
            assert block.start == 0
        else:
            assert block.start == blocks[index - 1].stop()


def test_state_mapping_keyframes_ordered() -> None:
    """Ensure keyframe blocks are ordered by identifier."""
    kf_a: KeyframeAttitude = KeyframeAttitude(
        keyframe_id=10,
        q_WB_wxyz=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
    )
    kf_b: KeyframeAttitude = KeyframeAttitude(
        keyframe_id=2,
        q_WB_wxyz=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
    )
    state: MountingState = MountingState.default().replace(keyframes=(kf_a, kf_b))
    mapping: StateMapping = StateMapping.from_state(
        state,
        include_translation_vars=False,
    )
    assert mapping.keyframe_block(2).dim == 3
    assert mapping.keyframe_block(10).dim == 3
    blocks = mapping.blocks()
    keyframe_names = [block.name for block in blocks if block.name.startswith("R_WB_")]
    assert keyframe_names == ["R_WB_2", "R_WB_10"]


def test_state_mapping_translation_blocks() -> None:
    """Ensure translation blocks are included when requested."""
    state: MountingState = MountingState.default()
    mapping: StateMapping = StateMapping.from_state(
        state,
        include_translation_vars=True,
    )
    assert mapping.has(BLOCK_NAME_P_BI)
    assert mapping.has(BLOCK_NAME_P_BM)
