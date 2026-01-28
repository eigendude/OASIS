################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""High-level configuration wrapper for mounting calibration."""

from __future__ import annotations

from dataclasses import dataclass

from .mounting_params import MountingParams
from .mounting_params import MountingParamsError


class MountingConfigError(Exception):
    """Raised when mounting configuration validation fails."""


@dataclass(frozen=True)
class MountingConfig:
    """Convenience wrapper around mounting parameters."""

    params: MountingParams

    def __init__(self, params: MountingParams) -> None:
        """Initialize the configuration wrapper and validate."""
        object.__setattr__(self, "params", params)
        self.validate()

    def validate(self) -> None:
        """Validate parameter invariants and cross-namespace policies."""
        try:
            self.params.validate()
        except MountingParamsError as exc:
            raise MountingConfigError(str(exc)) from exc

        if self.params.save.format not in {"yaml", "json"}:
            raise MountingConfigError("save.format must be 'yaml' or 'json'")

        if self.params.cluster.drop_policy not in {
            "redundant",
            "oldest",
            "lowest_information",
        }:
            raise MountingConfigError(
                "cluster.drop_policy must be redundant, oldest, "
                "or lowest_information"
            )

        if self.params.steady.window_type not in {"sliding", "tumbling"}:
            raise MountingConfigError("steady.window_type must be sliding or tumbling")

    def base_frame(self) -> str:
        """Return the configured base frame name."""
        return self.params.frames.base_frame

    def imu_raw_topic(self) -> str:
        """Return the configured IMU raw topic name."""
        return self.params.topics.imu_raw

    def mag_topic(self) -> str:
        """Return the configured magnetometer topic name."""
        return self.params.topics.magnetic_field
