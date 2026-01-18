################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

from __future__ import annotations

import pytest

from oasis_control.localization.ahrs.ahrs_error_state import AhrsErrorStateLayout
from oasis_control.localization.ahrs.ahrs_error_state import ErrorBlock
from oasis_control.localization.ahrs.ahrs_inject import inject_error_state
from oasis_control.localization.ahrs.ahrs_state import AhrsNominalState
from oasis_control.localization.ahrs.ahrs_state import default_nominal_state


def _default_state() -> AhrsNominalState:
    return default_nominal_state(
        body_frame_id="base_link",
        imu_frame_id="imu",
        mag_frame_id="mag",
    )


def test_inject_error_state_rejects_bad_aa_length() -> None:
    layout: AhrsErrorStateLayout = AhrsErrorStateLayout()
    layout.blocks["Aa"] = ErrorBlock(
        name=layout.blocks["Aa"].name,
        dim=8,
        start=layout.blocks["Aa"].start,
    )
    delta_x: list[float] = [0.0] * layout.dim

    with pytest.raises(ValueError, match="delta_aa must be length 9"):
        inject_error_state(layout, _default_state(), delta_x)


def test_inject_error_state_rejects_bad_xi_length() -> None:
    layout: AhrsErrorStateLayout = AhrsErrorStateLayout()
    layout.blocks["xi_bi"] = ErrorBlock(
        name=layout.blocks["xi_bi"].name,
        dim=5,
        start=layout.blocks["xi_bi"].start,
    )
    delta_x: list[float] = [0.0] * layout.dim

    with pytest.raises(ValueError, match="delta_xi_bi must be length 6"):
        inject_error_state(layout, _default_state(), delta_x)
