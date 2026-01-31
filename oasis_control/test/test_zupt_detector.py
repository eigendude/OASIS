################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for the ZUPT detector."""

from __future__ import annotations

import math
from typing import cast

import numpy as np

from oasis_control.localization.zupt_detector import ZuptDetector
from oasis_control.localization.zupt_detector import ZuptDetectorConfig


def _make_detector() -> ZuptDetector:
    config = ZuptDetectorConfig(
        min_stationary_sec=0.2,
        min_exit_sec=0.05,
        zupt_anneal_tau_sec=0.2,
    )
    return ZuptDetector(config)


def test_enters_stationary_with_quiet_gyro() -> None:
    detector = _make_detector()
    detector.set_duty_cycle(0.0)

    omega = np.zeros(3)
    cov = np.eye(3) * 0.01

    detector.update(0.0, omega, cov)
    result = detector.update(0.2, omega, cov)

    assert result["stationary"] is True
    assert result["should_publish_zupt"] is True


def test_does_not_enter_stationary_when_duty_nonzero() -> None:
    detector = _make_detector()
    detector.set_duty_cycle(0.2)

    omega = np.zeros(3)
    cov = np.eye(3) * 0.01

    detector.update(0.0, omega, cov)
    result = detector.update(0.2, omega, cov)

    assert result["stationary"] is False


def test_exits_stationary_when_duty_nonzero() -> None:
    detector = _make_detector()
    detector.set_duty_cycle(0.0)

    omega = np.zeros(3)
    cov = np.eye(3) * 0.01

    detector.update(0.0, omega, cov)
    detector.update(0.2, omega, cov)
    detector.set_duty_cycle(0.5)
    result = detector.update(0.26, omega, cov)

    assert result["stationary"] is False


def test_exits_stationary_when_gyro_loud() -> None:
    detector = _make_detector()
    detector.set_duty_cycle(0.0)

    quiet_omega = np.zeros(3)
    loud_omega = np.array([1.0, 0.0, 0.0])
    cov = np.eye(3) * 0.01

    detector.update(0.0, quiet_omega, cov)
    detector.update(0.2, quiet_omega, cov)
    result = detector.update(0.26, loud_omega, cov)

    assert result["stationary"] is False


def test_mahalanobis_gate_responds_to_tiny_covariance() -> None:
    detector = _make_detector()
    detector.set_duty_cycle(0.0)

    omega = np.array([0.2, 0.0, 0.0])
    cov = np.eye(3) * 1e-6

    result = detector.update(0.0, omega, cov)
    diagnostics = cast(dict[str, float], result["diagnostics"])

    assert diagnostics["d2"] > detector.state.last_gate_exit


def test_regularization_handles_singular_covariance() -> None:
    detector = _make_detector()
    detector.set_duty_cycle(0.0)

    omega = np.zeros(3)
    cov = np.zeros((3, 3))

    result = detector.update(0.0, omega, cov)
    diagnostics = cast(dict[str, float], result["diagnostics"])

    assert math.isfinite(diagnostics["d2"])


def test_annealing_decreases_variance_with_dwell() -> None:
    detector = _make_detector()
    detector.set_duty_cycle(0.0)

    omega = np.zeros(3)
    cov = np.eye(3) * 0.01

    detector.update(0.0, omega, cov)
    detector.update(0.2, omega, cov)
    result_start = detector.update(0.25, omega, cov)
    result_later = detector.update(0.45, omega, cov)

    var_start = cast(float, result_start["zupt_vx_variance"])
    var_later = cast(float, result_later["zupt_vx_variance"])

    assert isinstance(var_start, float)
    assert isinstance(var_later, float)
    assert var_later < var_start
