################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Update report types for mounting calibration."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class UpdateReport:
    """Summary of one update step in the mounting calibration pipeline.

    Attributes:
        did_update: True when the solver performed an update
        iterations: Number of solver iterations executed
        step_norm: Step size norm from the solver when available
        residual_rms: RMS of residuals when available
        need_more_tilt: True when more tilt diversity is required
        need_more_yaw: True when more yaw diversity is required
        dropped_segments: Number of segments dropped in this update
        dropped_mags: Number of mag samples dropped in this update
        message: Optional status message
    """

    did_update: bool
    iterations: int
    step_norm: float | None
    residual_rms: float | None
    need_more_tilt: bool
    need_more_yaw: bool
    dropped_segments: int
    dropped_mags: int
    message: str | None = None

    def __post_init__(self) -> None:
        """Validate update report fields."""
        if not isinstance(self.did_update, bool):
            raise ValueError("did_update must be a bool")
        if not isinstance(self.iterations, int) or isinstance(self.iterations, bool):
            raise ValueError("iterations must be an int")
        if self.iterations < 0:
            raise ValueError("iterations must be non-negative")
        if not isinstance(self.need_more_tilt, bool):
            raise ValueError("need_more_tilt must be a bool")
        if not isinstance(self.need_more_yaw, bool):
            raise ValueError("need_more_yaw must be a bool")
        if not isinstance(self.dropped_segments, int) or isinstance(
            self.dropped_segments, bool
        ):
            raise ValueError("dropped_segments must be an int")
        if self.dropped_segments < 0:
            raise ValueError("dropped_segments must be non-negative")
        if not isinstance(self.dropped_mags, int) or isinstance(
            self.dropped_mags, bool
        ):
            raise ValueError("dropped_mags must be an int")
        if self.dropped_mags < 0:
            raise ValueError("dropped_mags must be non-negative")
        if self.step_norm is not None:
            _require_finite_non_negative(self.step_norm, "step_norm")
        if self.residual_rms is not None:
            _require_finite_non_negative(self.residual_rms, "residual_rms")
        if self.message is not None and not isinstance(self.message, str):
            raise ValueError("message must be a str or None")


def _require_finite_non_negative(value: float, name: str) -> None:
    """Require a finite, non-negative scalar value."""
    if not np.isfinite(value):
        raise ValueError(f"{name} must be finite")
    if value < 0.0:
        raise ValueError(f"{name} must be non-negative")
