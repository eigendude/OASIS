################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for the velocity estimator."""

from __future__ import annotations

import math
from typing import cast

from oasis_control.localization.velocity_estimator import VelocityEstimator
from oasis_control.localization.velocity_estimator import VelocityEstimatorConfig


def _make_estimator() -> VelocityEstimator:
    config: VelocityEstimatorConfig = VelocityEstimatorConfig(
        process_noise_v=0.0,
        process_noise_b=0.0,
        initial_speed_sigma=0.1,
        initial_bias_sigma=0.1,
        max_dt_sec=0.2,
        min_dt_sec=0.0,
    )
    return VelocityEstimator(config)


def test_predict_integrates_forward_accel() -> None:
    estimator: VelocityEstimator = _make_estimator()

    estimator.predict(0.0, 1.0)
    estimator.predict(0.1, 1.0)

    v: float = estimator.get_velocity_mps()

    assert math.isclose(v, 0.1, rel_tol=1e-6, abs_tol=1e-9)


def test_predict_clamps_large_dt() -> None:
    estimator: VelocityEstimator = _make_estimator()

    estimator.predict(0.0, 1.0)
    diagnostics: dict[str, object] = estimator.predict(1.0, 1.0)

    dt_clamped: bool = cast(bool, diagnostics["dt_clamped"])
    v: float = estimator.get_velocity_mps()

    assert dt_clamped is True
    assert math.isclose(v, 0.0, rel_tol=1e-6, abs_tol=1e-9)


def test_zupt_update_reduces_speed() -> None:
    estimator: VelocityEstimator = _make_estimator()

    estimator.predict(0.0, 1.0)
    estimator.predict(0.1, 1.0)

    v_before: float = estimator.get_velocity_mps()
    estimator.update_zupt(0.1, 0.01)
    v_after: float = estimator.get_velocity_mps()

    assert abs(v_after) < abs(v_before)


def test_non_finite_predict_is_ignored() -> None:
    estimator: VelocityEstimator = _make_estimator()

    estimator.predict(0.0, 1.0)
    v_before: float = estimator.get_velocity_mps()

    estimator.predict(0.1, math.nan)
    v_after: float = estimator.get_velocity_mps()
    snapshot: dict[str, object] = estimator.get_diagnostics_snapshot()
    last_reason: str = cast(str, snapshot["last_reason"])

    assert math.isclose(v_before, v_after, rel_tol=1e-6, abs_tol=1e-9)
    assert last_reason == "predict_non_finite"
