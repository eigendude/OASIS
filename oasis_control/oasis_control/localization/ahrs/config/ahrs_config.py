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

from dataclasses import dataclass
from typing import Mapping

from oasis_control.localization.ahrs.config.ahrs_params import AhrsParams


@dataclass(frozen=True, slots=True)
class AhrsConfig:
    """High-level configuration container for the AHRS core.

    Responsibility:
        Provide a cohesive configuration object that aggregates AhrsParams
        and exposes settings to the filter, models, and timing components.

    Purpose:
        Aggregate all AHRS parameters into a single object passed throughout
        the core, ensuring validation and immutability.

    Inputs/outputs:
        - Inputs: AhrsParams or configuration dictionaries.
        - Outputs: validated, immutable configuration used by the core.

    Dependencies:
        - Uses AhrsParams for schema and validation.

    Public API (to be implemented):
        - from_params(params)
        - validate()
        - as_dict()

    Data contract:
        - Contains the full AhrsParams structure as `params`.
        - Converts t_buffer_sec once in configuration into t_buffer_ns
          deterministically.

    Frames and units:
        - Frame identifiers are stored as strings.
        - Numeric fields use Units-defined scales.

    Determinism and edge cases:
        - Configuration is immutable once constructed for deterministic
          behavior.
        - Construction must not read ROS params or clocks.
        - Invalid parameters should raise validation errors.

    Equations:
        - No equations; configuration only.

    Numerical stability notes:
        - Ensure derived covariances are SPD.

    Suggested unit tests:
        - as_dict round-trip preserves values.
        - validate rejects missing required fields.
    """

    params: AhrsParams
    t_buffer_ns: int

    @classmethod
    def from_params(cls, params: AhrsParams | Mapping[str, object]) -> AhrsConfig:
        """Construct a configuration from parameters or a mapping."""
        if isinstance(params, Mapping):
            params_obj: AhrsParams = AhrsParams.from_dict(params)
        elif isinstance(params, AhrsParams):
            params_obj = params
        else:
            raise ValueError("params must be AhrsParams or mapping")
        params_obj.validate()
        t_buffer_ns: int = cls._to_ns(params_obj.t_buffer_sec)
        config: AhrsConfig = cls(params=params_obj, t_buffer_ns=t_buffer_ns)
        config.validate()
        return config

    def validate(self) -> None:
        """Validate configuration and raise ValueError on failure."""
        self.params.validate()
        expected: int = self._to_ns(self.params.t_buffer_sec)
        if self.t_buffer_ns != expected:
            raise ValueError("t_buffer_ns must match t_buffer_sec")

    def as_dict(self) -> dict[str, object]:
        """Return a JSON-serializable dict representation."""
        return {
            "t_buffer_ns": self.t_buffer_ns,
            "params": self.params.as_dict(),
        }

    @staticmethod
    def _to_ns(t_seconds: float) -> int:
        return int(t_seconds * 1_000_000_000 + 0.5)
