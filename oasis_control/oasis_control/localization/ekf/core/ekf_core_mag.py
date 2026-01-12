################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""
Magnetometer update handling for the EKF core
"""

from __future__ import annotations

from typing import Optional

import numpy as np

from oasis_control.localization.ekf.core.ekf_core_utils import EkfCoreUtilsMixin
from oasis_control.localization.ekf.ekf_state import EkfState
from oasis_control.localization.ekf.ekf_types import EkfMatrix
from oasis_control.localization.ekf.ekf_types import EkfTime
from oasis_control.localization.ekf.ekf_types import EkfUpdateData
from oasis_control.localization.ekf.ekf_types import MagSample
from oasis_control.localization.ekf.models.mag_measurement_model import (
    MagMeasurementModel,
)
from oasis_control.localization.ekf.se3 import quat_to_rotation_matrix


class EkfCoreMagMixin(EkfCoreUtilsMixin):
    _mag_model: MagMeasurementModel
    _mag_alpha: float
    _mag_r_min: np.ndarray
    _mag_r_max: np.ndarray
    _mag_r0_default: np.ndarray
    _state: EkfState

    def update_with_mag(self, mag_sample: MagSample, t_meas: EkfTime) -> EkfUpdateData:
        """
        Apply the magnetometer update model
        """

        z_dim: int = 3
        z: np.ndarray = np.asarray(mag_sample.magnetic_field_t, dtype=float)
        r: np.ndarray = self._ensure_mag_covariance(mag_sample)
        rot_bm: np.ndarray = quat_to_rotation_matrix(
            self._state.extrinsic_bm.rotation_wxyz
        )
        self._mag_model.set_mag_to_body_rotation(rot_bm)
        z_hat: np.ndarray
        h: np.ndarray
        state_legacy: np.ndarray = self._state.legacy_state()
        z_hat, h = self._mag_model.linearize(state_legacy)
        h = self._state.lift_legacy_jacobian(h)
        residual: np.ndarray = z - z_hat
        s_hat: np.ndarray = h @ self._state.covariance @ h.T
        s: np.ndarray = s_hat + r
        s = 0.5 * (s + s.T)
        base: float = 1e-12
        scale: float = max(1.0, float(np.max(np.abs(np.diag(s)))))
        jitter: float = base * scale
        s = s + jitter * np.eye(s.shape[0], dtype=float)
        l: Optional[np.ndarray]
        try:
            l = np.linalg.cholesky(s)
        except np.linalg.LinAlgError:
            l = None
            for factor in (1e-10, 1e-8, 1e-6, 1e-4):
                s_try: np.ndarray = 0.5 * (s + s.T) + (factor * scale) * np.eye(
                    s.shape[0], dtype=float
                )
                try:
                    l = np.linalg.cholesky(s_try)
                    s = s_try
                    break
                except np.linalg.LinAlgError:
                    continue
            else:
                mag_reject_update: EkfUpdateData = EkfUpdateData(
                    sensor="magnetic_field",
                    frame_id=mag_sample.frame_id,
                    t_meas=t_meas,
                    accepted=False,
                    reject_reason="singular S",
                    z_dim=z_dim,
                    z=z.tolist(),
                    z_hat=z_hat.tolist(),
                    nu=residual.tolist(),
                    r=EkfMatrix(rows=z_dim, cols=z_dim, data=r.flatten().tolist()),
                    s_hat=EkfMatrix(
                        rows=z_dim, cols=z_dim, data=s_hat.flatten().tolist()
                    ),
                    s=EkfMatrix(rows=z_dim, cols=z_dim, data=s.flatten().tolist()),
                    maha_d2=0.0,
                    gate_d2_threshold=0.0,
                    reproj_rms_px=0.0,
                )
                return mag_reject_update

        y: np.ndarray = np.linalg.solve(l, residual)
        x: np.ndarray = np.linalg.solve(l.T, y)
        maha_d2: float = float(residual.T @ x)
        gate_d2_threshold: float = 0.0

        ph_t: np.ndarray = self._state.covariance @ h.T
        tmp: np.ndarray = np.linalg.solve(l, ph_t.T)
        s_inv_ph_t: np.ndarray = np.linalg.solve(l.T, tmp)
        k_gain: np.ndarray = s_inv_ph_t.T

        delta: np.ndarray = k_gain @ residual
        self._state.apply_delta(delta)
        identity: np.ndarray = np.eye(self._state.index.total_dim, dtype=float)
        temp: np.ndarray = identity - k_gain @ h
        self._state.covariance = (
            temp @ self._state.covariance @ temp.T + k_gain @ r @ k_gain.T
        )
        self._state.covariance = 0.5 * (
            self._state.covariance + self._state.covariance.T
        )
        self._update_mag_covariance(residual, s_hat)

        mag_accept_update: EkfUpdateData = EkfUpdateData(
            sensor="magnetic_field",
            frame_id=mag_sample.frame_id,
            t_meas=t_meas,
            accepted=True,
            reject_reason="",
            z_dim=z_dim,
            z=z.tolist(),
            z_hat=z_hat.tolist(),
            nu=residual.tolist(),
            r=EkfMatrix(rows=z_dim, cols=z_dim, data=r.flatten().tolist()),
            s_hat=EkfMatrix(rows=z_dim, cols=z_dim, data=s_hat.flatten().tolist()),
            s=EkfMatrix(rows=z_dim, cols=z_dim, data=s.flatten().tolist()),
            maha_d2=maha_d2,
            gate_d2_threshold=gate_d2_threshold,
            reproj_rms_px=0.0,
        )

        return mag_accept_update

    def _ensure_mag_covariance(self, mag_sample: MagSample) -> np.ndarray:
        if self._state.mag_covariance_initialized:
            return self._state.mag_covariance
        r_init: np.ndarray = self._mag_r0_default.copy()
        r_candidate: Optional[np.ndarray] = self._validated_mag_covariance(mag_sample)
        if r_candidate is not None:
            r_init = r_candidate
        self._state.mag_covariance = r_init
        self._state.mag_covariance_initialized = True
        return r_init

    def _validated_mag_covariance(
        self, mag_sample: MagSample
    ) -> Optional[np.ndarray]:
        r_raw: np.ndarray = np.asarray(mag_sample.magnetic_field_cov, dtype=float)
        if r_raw.size != 9:
            return None
        r_candidate: np.ndarray = r_raw.reshape(3, 3)
        if not np.all(np.isfinite(r_candidate)):
            return None
        if np.any(np.diag(r_candidate) <= 0.0):
            return None
        r_sym: np.ndarray = 0.5 * (r_candidate + r_candidate.T)
        try:
            np.linalg.cholesky(r_sym)
        except np.linalg.LinAlgError:
            return None
        return r_sym

    def _update_mag_covariance(self, residual: np.ndarray, s_hat: np.ndarray) -> None:
        if not self._state.mag_covariance_initialized:
            return
        residual_vec: np.ndarray = residual.reshape(3)
        r_prior: np.ndarray = self._state.mag_covariance
        residual_outer: np.ndarray = np.outer(residual_vec, residual_vec)
        r_update: np.ndarray = (1.0 - self._mag_alpha) * r_prior + self._mag_alpha * (
            residual_outer - s_hat
        )
        self._state.mag_covariance = self._clamp_spd(
            r_update, r_min=self._mag_r_min, r_max=self._mag_r_max
        )

    def _clamp_spd(
        self, matrix: np.ndarray, *, r_min: np.ndarray, r_max: np.ndarray
    ) -> np.ndarray:
        sym: np.ndarray = 0.5 * (matrix + matrix.T)
        evals: np.ndarray
        evecs: np.ndarray
        evals, evecs = np.linalg.eigh(sym)
        min_eig: float = float(np.min(np.linalg.eigvalsh(r_min)))
        max_eig: float = float(np.max(np.linalg.eigvalsh(r_max)))

        # Unitless eigenvalue floor to keep SPD when bounds are degenerate
        min_eig = max(min_eig, 1.0e-18)
        evals_clamped: np.ndarray = np.clip(evals, min_eig, max_eig)
        clamped: np.ndarray = evecs @ np.diag(evals_clamped) @ evecs.T
        return 0.5 * (clamped + clamped.T)

    def build_rejected_mag_update(
        self, mag_sample: MagSample, t_meas: EkfTime, *, reject_reason: str
    ) -> EkfUpdateData:
        return self._build_rejected_mag_update(
            mag_sample, t_meas, reject_reason=reject_reason
        )

    def _build_rejected_mag_update(
        self, mag_sample: MagSample, t_meas: EkfTime, *, reject_reason: str
    ) -> EkfUpdateData:
        z_dim: int = 3
        nan_values: list[float] = self._nan_list(z_dim)
        zero_matrix: EkfMatrix = EkfMatrix(rows=z_dim, cols=z_dim, data=[0.0] * 9)
        r: EkfMatrix = EkfMatrix(
            rows=z_dim, cols=z_dim, data=list(mag_sample.magnetic_field_cov)
        )
        return EkfUpdateData(
            sensor="magnetic_field",
            frame_id=mag_sample.frame_id,
            t_meas=t_meas,
            accepted=False,
            reject_reason=reject_reason,
            z_dim=z_dim,
            z=list(mag_sample.magnetic_field_t),
            z_hat=nan_values,
            nu=nan_values,
            r=r,
            s_hat=zero_matrix,
            s=zero_matrix,
            maha_d2=0.0,
            gate_d2_threshold=0.0,
            reproj_rms_px=0.0,
        )
