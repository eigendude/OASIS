################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for mounting parameter schema."""

from __future__ import annotations

import dataclasses
from typing import cast

import numpy as np
import pytest

from oasis_control.localization.mounting.config.mounting_params import BootstrapParams
from oasis_control.localization.mounting.config.mounting_params import MagParams
from oasis_control.localization.mounting.config.mounting_params import MountingParams
from oasis_control.localization.mounting.config.mounting_params import (
    MountingParamsError,
)
from oasis_control.localization.mounting.config.mounting_params import MountParams
from oasis_control.localization.mounting.config.mounting_params import SaveParams
from oasis_control.localization.mounting.config.mounting_params import SteadyParams


def test_defaults_validate() -> None:
    """Defaults should validate successfully."""
    params: MountingParams = MountingParams.defaults()
    params.validate()


def test_numpy_coercion_shapes() -> None:
    """Array-like inputs should coerce to float64 shape (3,) arrays."""
    bi_prior: np.ndarray = cast(np.ndarray, [1.0, 2.0, 3.0])
    bm_prior: np.ndarray = cast(np.ndarray, [4.0, 5.0, 6.0])
    rm_init: np.ndarray = cast(np.ndarray, [1e-3, 2e-3, 3e-3])
    rm_min: np.ndarray = cast(np.ndarray, [1e-5, 2e-5, 3e-5])
    rm_max: np.ndarray = cast(np.ndarray, [1e-1, 2e-1, 3e-1])
    mount_params: MountParams = MountParams(
        p_BI_prior_m=bi_prior,
        p_BM_prior_m=bm_prior,
        baseline_im_to_mag_m=1.0,
        baseline_hard_enabled=False,
    )
    mag_params: MagParams = MagParams(
        Rm_init_diag=rm_init,
        Rm_min_diag=rm_min,
        Rm_max_diag=rm_max,
        disturbance_gate_sigma=1.0,
    )
    params: MountingParams = MountingParams.defaults().replace(
        mount=mount_params,
        mag=mag_params,
    )

    assert isinstance(params.mount.p_BI_prior_m, np.ndarray)
    assert params.mount.p_BI_prior_m.shape == (3,)
    assert isinstance(params.mount.p_BM_prior_m, np.ndarray)
    assert params.mount.p_BM_prior_m.shape == (3,)
    assert isinstance(params.mag.Rm_init_diag, np.ndarray)
    assert params.mag.Rm_init_diag.shape == (3,)
    assert isinstance(params.mag.Rm_min_diag, np.ndarray)
    assert params.mag.Rm_min_diag.shape == (3,)
    assert isinstance(params.mag.Rm_max_diag, np.ndarray)
    assert params.mag.Rm_max_diag.shape == (3,)


def test_baseline_validation() -> None:
    """Baseline validation should enforce hard constraint rules."""
    params: MountingParams = MountingParams.defaults()

    enabled_none: MountingParams = params.replace(
        mount=dataclasses.replace(
            params.mount,
            baseline_hard_enabled=True,
            baseline_im_to_mag_m=None,
        )
    )
    with pytest.raises(MountingParamsError):
        enabled_none.validate()

    enabled_invalid: MountingParams = params.replace(
        mount=dataclasses.replace(
            params.mount,
            baseline_hard_enabled=True,
            baseline_im_to_mag_m=0.0,
        )
    )
    with pytest.raises(MountingParamsError):
        enabled_invalid.validate()

    disabled_none: MountingParams = params.replace(
        mount=dataclasses.replace(
            params.mount,
            baseline_hard_enabled=False,
            baseline_im_to_mag_m=None,
        )
    )
    disabled_none.validate()

    disabled_invalid: MountingParams = params.replace(
        mount=dataclasses.replace(
            params.mount,
            baseline_hard_enabled=False,
            baseline_im_to_mag_m=-1.0,
        )
    )
    with pytest.raises(MountingParamsError):
        disabled_invalid.validate()


def test_basic_numeric_validations() -> None:
    """Numeric validation should reject non-positive values."""
    params: MountingParams = MountingParams.defaults()

    invalid_bootstrap: MountingParams = params.replace(
        bootstrap=BootstrapParams(
            bootstrap_sec=0.0,
            require_flat_stationary=params.bootstrap.require_flat_stationary,
            mag_reference_required=params.bootstrap.mag_reference_required,
        )
    )
    with pytest.raises(MountingParamsError):
        invalid_bootstrap.validate()

    invalid_steady: MountingParams = params.replace(
        steady=SteadyParams(
            steady_sec=0.0,
            window_type=params.steady.window_type,
        )
    )
    with pytest.raises(MountingParamsError):
        invalid_steady.validate()

    invalid_save: MountingParams = params.replace(
        save=SaveParams(
            save_period_sec=0.0,
            output_path=params.save.output_path,
            format=params.save.format,
            atomic_write=params.save.atomic_write,
        )
    )
    with pytest.raises(MountingParamsError):
        invalid_save.validate()

    invalid_mag: MountingParams = params.replace(
        mag=dataclasses.replace(params.mag, s_min_T=0.0)
    )
    with pytest.raises(MountingParamsError):
        invalid_mag.validate()


def test_replace_helper_immutability() -> None:
    """Replace should return a new instance without mutating the original."""
    params: MountingParams = MountingParams.defaults()
    updated: MountingParams = params.replace(
        frames=dataclasses.replace(params.frames, base_frame="base")
    )

    assert params.frames.base_frame == "base_link"
    assert updated.frames.base_frame == "base"
