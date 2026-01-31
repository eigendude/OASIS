################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for the speedometer core wrapper."""

from __future__ import annotations

import math
from typing import cast

from oasis_control.localization.speedometer_core import SpeedometerCore
from oasis_control.localization.speedometer_core import SpeedometerCoreConfig


def _make_core(max_dt_sec: float = 0.2) -> SpeedometerCore:
    config: SpeedometerCoreConfig = SpeedometerCoreConfig(
        process_noise_v=0.0,
        process_noise_b=0.0,
        initial_speed_sigma=0.1,
        initial_bias_sigma=0.1,
        max_dt_sec=max_dt_sec,
        min_dt_sec=0.0,
    )
    return SpeedometerCore(config)


def test_predict_integrates_accel() -> None:
    core: SpeedometerCore = _make_core()

    core.predict(0.0, 1.0)
    core.predict(0.1, 1.0)

    v: float = core.get_velocity_mps()

    assert math.isclose(v, 0.1, rel_tol=1e-6, abs_tol=1e-9)


def test_zupt_reduces_speed_and_updates_bias() -> None:
    core: SpeedometerCore = _make_core()

    core.predict(0.0, 1.0)
    core.predict(0.1, 1.0)

    v_before: float = core.get_velocity_mps()
    core.apply_zupt(0.1, 0.01)
    v_after: float = core.get_velocity_mps()
    snapshot: dict[str, object] = core.snapshot()
    bias: float = cast(float, snapshot["b_mps2"])

    assert abs(v_after) < abs(v_before)
    assert math.isfinite(bias)


def test_bad_dt_is_clamped() -> None:
    core: SpeedometerCore = _make_core(max_dt_sec=0.2)

    core.predict(0.0, 1.0)
    diagnostics: dict[str, object] = core.predict(1.0, 1.0)

    dt_clamped: bool = cast(bool, diagnostics["dt_clamped"])
    v: float = core.get_velocity_mps()

    assert dt_clamped is True
    assert math.isclose(v, 0.0, rel_tol=1e-6, abs_tol=1e-9)


def test_priors_validation() -> None:
    core: SpeedometerCore = _make_core()

    snapshot_before: dict[str, object] = core.snapshot()
    b_before: float = cast(float, snapshot_before["b_mps2"])
    p_before: float = cast(float, snapshot_before["P_bb"])

    core.set_bias_priors(math.nan, -1.0)

    snapshot_after: dict[str, object] = core.snapshot()
    b_after: float = cast(float, snapshot_after["b_mps2"])
    p_after: float = cast(float, snapshot_after["P_bb"])

    assert math.isclose(b_before, b_after, rel_tol=1e-6, abs_tol=1e-9)
    assert math.isclose(p_before, p_after, rel_tol=1e-6, abs_tol=1e-9)

    core.set_bias_priors(0.5, 0.25)

    snapshot_valid: dict[str, object] = core.snapshot()
    b_valid: float = cast(float, snapshot_valid["b_mps2"])
    p_valid: float = cast(float, snapshot_valid["P_bb"])

    assert math.isclose(b_valid, 0.5, rel_tol=1e-6, abs_tol=1e-9)
    assert math.isclose(p_valid, 0.25, rel_tol=1e-6, abs_tol=1e-9)
