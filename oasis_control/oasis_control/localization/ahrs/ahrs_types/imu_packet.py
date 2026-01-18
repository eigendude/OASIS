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

import hashlib
import json
from dataclasses import dataclass
from dataclasses import field
from typing import Mapping
from typing import Sequence

from oasis_control.localization.ahrs.math_utils.linalg import LinearAlgebra


@dataclass(frozen=True, slots=True)
class ImuPacket:
    """IMU measurement packet for gyro and accelerometer updates.

    Data contract:
        t_meas_ns:
            Measurement timestamp in int nanoseconds since an arbitrary epoch
            used for exact time alignment
        frame_id:
            Sensor frame identifier for {I} used to label z_omega and z_accel
        z_omega:
            Gyro measurement vector (rad/s) in frame {I}
        R_omega:
            Gyro measurement covariance matrix (rad/s)^2, full 3x3 SPD
        z_accel:
            Accel measurement vector (m/s^2) in frame {I} for specific force
        R_accel:
            Accel measurement covariance matrix (m/s^2)^2, full 3x3 SPD
        calibration_prior:
            ExactTime-paired calibration payload for the same t_meas_ns
        calibration_meta:
            Deterministic metadata describing calibration source and validity
        calibration_fingerprint:
            Deterministic SHA-256 fingerprint of calibration_prior content

    Determinism and edge cases:
        - ExactTime pairing requires calibration_prior to be present for the
          same t_meas_ns
        - Hashing excludes nondeterministic metadata such as receipt time
        - Covariances must be full SPD matrices and are never diagonalized
    """

    t_meas_ns: int
    frame_id: str
    z_omega: list[float]
    R_omega: list[list[float]]
    z_accel: list[float]
    R_accel: list[list[float]]
    calibration_prior: Mapping[str, object] | None
    calibration_meta: Mapping[str, object]
    calibration_fingerprint: str = field(init=False)

    def __post_init__(self) -> None:
        """Compute the deterministic calibration fingerprint"""
        fingerprint: str = ""
        if self.calibration_prior is not None:
            fingerprint = self._fingerprint_payload(self.calibration_prior)
        object.__setattr__(self, "calibration_fingerprint", fingerprint)
        self.validate()

    def validate(self) -> None:
        """Validate packet fields and raise ValueError on failure."""
        if not self._is_int(self.t_meas_ns):
            raise ValueError("t_meas_ns must be int nanoseconds")
        if not self.frame_id:
            raise ValueError("frame_id must be a non-empty string")
        self._validate_vector("z_omega", self.z_omega, 3)
        self._validate_matrix("R_omega", self.R_omega, 3)
        if not LinearAlgebra.is_spd(self.R_omega):
            raise ValueError("R_omega must be SPD")
        self._validate_vector("z_accel", self.z_accel, 3)
        self._validate_matrix("R_accel", self.R_accel, 3)
        if not LinearAlgebra.is_spd(self.R_accel):
            raise ValueError("R_accel must be SPD")
        if self.calibration_prior is None:
            raise ValueError("missing calibration_prior for ExactTime sync")
        if not isinstance(self.calibration_prior, Mapping):
            raise ValueError("calibration_prior must be a mapping")
        self._normalize_json_payload(
            "calibration_prior",
            self.calibration_prior,
        )
        if not isinstance(self.calibration_meta, Mapping):
            raise ValueError("calibration_meta must be a mapping")
        self._normalize_json_payload("calibration_meta", self.calibration_meta)

    def as_dict(self) -> dict[str, object]:
        """Return a JSON-serializable dict representation."""
        calibration_prior: object
        if self.calibration_prior is None:
            calibration_prior = None
        else:
            calibration_prior = self._normalize_json_payload(
                "calibration_prior",
                self.calibration_prior,
            )
        calibration_meta: object = self._normalize_json_payload(
            "calibration_meta",
            self.calibration_meta,
        )
        return {
            "t_meas_ns": self.t_meas_ns,
            "frame_id": self.frame_id,
            "z_omega": list(self.z_omega),
            "R_omega": [list(row) for row in self.R_omega],
            "z_accel": list(self.z_accel),
            "R_accel": [list(row) for row in self.R_accel],
            "calibration_prior": calibration_prior,
            "calibration_meta": calibration_meta,
            "calibration_fingerprint": self.calibration_fingerprint,
        }

    @staticmethod
    def _is_int(value: object) -> bool:
        return isinstance(value, int) and not isinstance(value, bool)

    @staticmethod
    def _validate_vector(name: str, vector: Sequence[float], length: int) -> None:
        if len(vector) != length:
            raise ValueError(f"{name} must have length {length}")

    @staticmethod
    def _validate_matrix(
        name: str, matrix: Sequence[Sequence[float]], size: int
    ) -> None:
        if len(matrix) != size:
            raise ValueError(f"{name} must be {size}x{size}")
        for row in matrix:
            if len(row) != size:
                raise ValueError(f"{name} must be {size}x{size}")

    @classmethod
    def _normalize_json_payload(cls, name: str, payload: object) -> object:
        if payload is None:
            return None
        if isinstance(payload, (str, int, float, bool)):
            return payload
        if isinstance(payload, Mapping):
            normalized: dict[str, object] = {}
            for key, value in payload.items():
                if not isinstance(key, str):
                    raise ValueError(f"{name} has non-string key")
                normalized[key] = cls._normalize_json_payload(name, value)
            return normalized
        if isinstance(payload, Sequence) and not isinstance(payload, (str, bytes)):
            return [cls._normalize_json_payload(name, item) for item in payload]
        raise ValueError(f"{name} is not JSON-serializable")

    @classmethod
    def _fingerprint_payload(cls, payload: Mapping[str, object]) -> str:
        normalized: object = cls._normalize_json_payload("calibration_prior", payload)
        canonical: str = json.dumps(
            normalized,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=True,
        )
        return hashlib.sha256(canonical.encode("utf-8")).hexdigest()
