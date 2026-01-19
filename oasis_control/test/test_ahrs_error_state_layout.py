################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

from oasis_control.localization.ahrs.ahrs_error_state import AhrsErrorStateLayout
from oasis_control.localization.ahrs.ahrs_error_state import ErrorBlock
from oasis_control.localization.ahrs.ahrs_error_state import error_state_names
from oasis_control.localization.ahrs.ahrs_state import AhrsNominalState
from oasis_control.localization.ahrs.ahrs_state import default_nominal_state
from oasis_control.localization.ahrs.ahrs_state import validate_nominal_state


def test_layout_dim_matches_names_length() -> None:
    layout: AhrsErrorStateLayout = AhrsErrorStateLayout()
    names: list[str] = error_state_names(layout)

    assert len(names) == layout.dim


def test_layout_blocks_are_contiguous_and_non_overlapping() -> None:
    layout: AhrsErrorStateLayout = AhrsErrorStateLayout()
    current: int = 0

    for key in layout.order:
        block: ErrorBlock = layout.blocks[key]
        assert block.start == current
        current = block.start + block.dim

    assert current == layout.dim


def test_specific_block_dims() -> None:
    layout: AhrsErrorStateLayout = AhrsErrorStateLayout()

    assert layout.blocks["p"].dim == 3
    assert layout.blocks["v"].dim == 3
    assert layout.blocks["theta"].dim == 3
    assert layout.blocks["omega"].dim == 3
    assert layout.blocks["bg"].dim == 3
    assert layout.blocks["ba"].dim == 3
    assert layout.blocks["Aa"].dim == 9
    assert layout.blocks["xi_bi"].dim == 6
    assert layout.blocks["xi_bm"].dim == 6
    assert layout.blocks["g"].dim == 3
    assert layout.blocks["m"].dim == 3


def test_default_nominal_state_shapes() -> None:
    state: AhrsNominalState = default_nominal_state(
        body_frame_id="base_link",
        imu_frame_id="imu",
        mag_frame_id="mag",
    )

    validate_nominal_state(state)

    assert state.t_bi.parent_frame == "base_link"
    assert state.t_bi.child_frame == "imu"
    assert state.t_bm.parent_frame == "base_link"
    assert state.t_bm.child_frame == "mag"
