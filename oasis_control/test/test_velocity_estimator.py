################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for the forward velocity estimator."""

from __future__ import annotations

from typing import cast

import numpy as np

from oasis_control.localization.velocity_estimator import VelocityEstimator
from oasis_control.localization.velocity_estimator import VelocityEstimatorConfig


def _make_estimator() -> VelocityEstimator:
    config: VelocityEstimatorConfig = VelocityEstimatorConfig(
        process_noise_v=0.05,
        process_noise_b=0.01,
        initial_speed_sigma=0.2,
        initial_bias_sigma=0.1,
        max_dt_sec=0.5,
    )
    return VelocityEstimator(config)


def test_predict_integrates_forward_acceleration() -> None:
    estimator: VelocityEstimator = _make_estimator()

    estimator.predict(0.0, 1.0)
    diagnostics: dict[str, object] = estimator.predict(0.1, 1.0)

    v: float = cast(float, diagnostics["v"])

    assert np.isfinite(v)
    assert np.isclose(v, 0.1)


def test_zupt_update_reduces_velocity() -> None:
    estimator: VelocityEstimator = _make_estimator()

    estimator.predict(0.0, 1.0)
    estimator.predict(0.1, 1.0)

    before: float = estimator.get_velocity_mps()
    diagnostics: dict[str, object] = estimator.update_zupt(1.0, 1e-4)

    after: float = cast(float, diagnostics["v"])
    bias: float = cast(float, diagnostics["b"])

    assert abs(after) < abs(before)
    assert np.isfinite(bias)
    assert abs(bias) > 1e-9


def test_predict_clamps_large_dt() -> None:
    config: VelocityEstimatorConfig = VelocityEstimatorConfig(max_dt_sec=0.1)
    estimator: VelocityEstimator = VelocityEstimator(config)

    estimator.predict(0.0, 0.0)
    diagnostics: dict[str, object] = estimator.predict(1.0, 0.0)

    dt: float = cast(float, diagnostics["dt_sec"])
    dt_clamped: bool = cast(bool, diagnostics["dt_clamped"])

    assert dt == 0.0
    assert dt_clamped is True


def test_predict_skips_nonfinite_input() -> None:
    estimator: VelocityEstimator = _make_estimator()

    estimator.predict(0.0, 0.0)
    estimator.predict(1.0, 0.0)

    nan_value: float = float("nan")
    diagnostics: dict[str, object] = estimator.predict(2.0, nan_value)
    last_reason: str = estimator.state.last_reason

    assert last_reason == "predict_nonfinite"
    assert np.isfinite(cast(float, diagnostics["v"]))
