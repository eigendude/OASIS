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

import math
from typing import List
from typing import Mapping
from typing import Protocol
from typing import Sequence

from oasis_control.localization.ahrs.ahrs_types.imu_packet import ImuPacket
from oasis_control.localization.ahrs.ahrs_types.mag_packet import MagPacket
from oasis_control.localization.ahrs.ahrs_types.stationary_packet import (
    StationaryPacket,
)
from oasis_control.localization.ahrs.config.ahrs_config import AhrsConfig
from oasis_control.localization.ahrs.config.ahrs_params import AhrsParams
from oasis_control.localization.ahrs.filter.predict_step import PredictStep
from oasis_control.localization.ahrs.filter.update_step import UpdateReport
from oasis_control.localization.ahrs.filter.update_step import UpdateStep
from oasis_control.localization.ahrs.models.imu_model import ImuModel
from oasis_control.localization.ahrs.models.mag_model import MagModel
from oasis_control.localization.ahrs.models.stationary_model import StationaryModel
from oasis_control.localization.ahrs.state.ahrs_state import AhrsState
from oasis_control.localization.ahrs.state.covariance import AhrsCovariance
from oasis_control.localization.ahrs.state.state_mapping import StateMapping
from oasis_control.localization.ahrs.timing.time_base import TimeBase


class ImuModelProtocol(Protocol):
    """Protocol for IMU measurement model interactions."""

    def predict_gyro(self, state: AhrsState) -> List[float]: ...

    def residual_gyro(
        self, z_omega: Sequence[float], z_hat: Sequence[float]
    ) -> List[float]: ...

    def jacobian_gyro(self, state: AhrsState) -> List[List[float]]: ...

    def predict_accel(self, state: AhrsState) -> List[float]: ...

    def residual_accel(
        self, z_accel: Sequence[float], z_hat: Sequence[float]
    ) -> List[float]: ...

    def jacobian_accel(self, state: AhrsState) -> List[List[float]]: ...


class MagModelProtocol(Protocol):
    """Protocol for magnetometer measurement model interactions."""

    def predict(self, state: AhrsState) -> List[float]: ...

    def residual(self, z_m: Sequence[float], z_hat: Sequence[float]) -> List[float]: ...

    def jacobian(self, state: AhrsState) -> List[List[float]]: ...


class StationaryModelProtocol(Protocol):
    """Protocol for stationary measurement model interactions."""

    def predict_no_turn(self, state: AhrsState) -> List[float]: ...

    def residual_no_turn(
        self, z_omega: Sequence[float], z_hat: Sequence[float]
    ) -> List[float]: ...

    def jacobian_no_turn(self, state: AhrsState) -> List[List[float]]: ...

    def predict_zupt(self, state: AhrsState) -> List[float]: ...

    def residual_zupt(
        self, z_v: Sequence[float], z_hat: Sequence[float]
    ) -> List[float]: ...

    def jacobian_zupt(self, state: AhrsState) -> List[List[float]]: ...


class UpdateStepProtocol(Protocol):
    """Protocol for EKF update step implementations."""

    def update(
        self,
        state: AhrsState,
        covariance: AhrsCovariance,
        H: Sequence[Sequence[float]],
        R: Sequence[Sequence[float]],
        nu: Sequence[float],
    ) -> tuple[AhrsState, AhrsCovariance, UpdateReport]: ...


class AhrsEkf:
    """AHRS EKF orchestrator and measurement ordering rules.

    Responsibility:
        Coordinate prediction and measurement updates while enforcing
        deterministic update ordering and interface contracts for the AHRS
        core.

    Purpose:
        Provide a single entry point for prediction and measurement updates
        that respects deterministic ordering and measurement contracts.

    Inputs/outputs:
        - Inputs: time steps, ImuPacket, MagPacket, and configuration.
        - Outputs: updated AhrsState, AhrsCovariance, and UpdateReport entries.

    Dependencies:
        - Depends on PredictStep, UpdateStep, and measurement models.

    Public API (to be implemented):
        - predict(dt_ns)
        - update_gyro(imu_packet)
        - update_accel(imu_packet)
        - update_mag(mag_packet)
        - update_no_turn(stationary_packet)
        - update_zupt(stationary_packet)
        - reset()
        - state()
        - covariance()

    Data contract:
        - Maintains an AhrsState and AhrsCovariance instance.
        - Accepts ImuPacket and MagPacket measurements.
        - Produces UpdateReport entries per measurement.

    Frames and units:
        - gyro/accel: {I}
        - mag: {M}
        - zupt: {W}
        - no_turn: {B}
        - State units follow Units.

    Determinism and edge cases:
        - Enforce deterministic update order at identical timestamps:
            1) priors once
            2) gyro update
            3) accel update
            4) mag update
            5) if stationary packet present and is_stationary:
               5a) no-turn update (if enabled)
               5b) ZUPT update
        - At a given timestamp, apply updates in the specified order.
        - Do not treat IMU or mag samples as process inputs.

    Equations:
        EKF update equations:
            S = H P Hᵀ + R
            K = P Hᵀ S⁻¹
            δx = K ν
            x ← x ⊕ δx
            P ← (I - K H) P (I - K H)ᵀ + K R Kᵀ

    Numerical stability notes:
        - Symmetrize covariance after each update.
        - Reject updates when S is not SPD.

    Suggested unit tests:
        - Deterministic ordering of updates at same timestamp.
        - Update reports capture accepted/rejected status.
    """

    def __init__(
        self,
        *,
        state: AhrsState | None = None,
        covariance: AhrsCovariance | None = None,
        config: AhrsConfig | None = None,
        Q_c: Sequence[Sequence[float]] | None = None,
        enable_no_turn: bool = True,
        imu_model: ImuModelProtocol | None = None,
        mag_model: MagModelProtocol | None = None,
        stationary_model: StationaryModelProtocol | None = None,
        update_step: UpdateStepProtocol | None = None,
    ) -> None:
        self._state: AhrsState = state if state is not None else AhrsState.reset()
        self._covariance: AhrsCovariance = (
            covariance
            if covariance is not None
            else AhrsCovariance.from_matrix(_identity(StateMapping.dimension()))
        )
        self._config: AhrsConfig | None = config
        self._Q_c: List[List[float]]
        if Q_c is not None:
            self._Q_c = _copy_matrix(Q_c)
        else:
            params: AhrsParams = (
                config.params if config is not None else AhrsParams.defaults()
            )
            self._Q_c = _build_process_noise(params)
        self._enable_no_turn: bool = enable_no_turn
        self._imu_model: ImuModelProtocol = (
            imu_model if imu_model is not None else ImuModel()
        )
        self._mag_model: MagModelProtocol = (
            mag_model if mag_model is not None else MagModel()
        )
        self._stationary_model: StationaryModelProtocol = (
            stationary_model if stationary_model is not None else StationaryModel()
        )
        self._update_step: UpdateStepProtocol = (
            update_step if update_step is not None else UpdateStep()
        )
        self._last_time_ns: int = 0
        self.last_reports: dict[str, UpdateReport] = {}
        self._calibration_prior_applied: bool = False

    def propagate_to(self, t_ns: int) -> None:
        """Propagate the filter to the requested timestamp."""
        TimeBase.validate_monotonic(self._last_time_ns, t_ns)
        dt_ns: int = t_ns - self._last_time_ns
        if dt_ns == 0:
            return
        self._state, self._covariance = PredictStep.propagate(
            self._state,
            self._covariance,
            dt_ns,
            self._Q_c,
        )
        self._last_time_ns = t_ns

    def update_imu(self, imu_packet: ImuPacket) -> None:
        """Apply gyro then accel updates from the IMU packet."""
        self._apply_calibration_prior_once(imu_packet)
        z_hat_omega: List[float] = self._imu_model.predict_gyro(self._state)
        nu_omega: List[float] = self._imu_model.residual_gyro(
            imu_packet.z_omega,
            z_hat_omega,
        )
        H_omega: List[List[float]] = self._imu_model.jacobian_gyro(self._state)
        self._state, self._covariance, report_gyro = self._update_step.update(
            self._state,
            self._covariance,
            H_omega,
            imu_packet.R_omega,
            nu_omega,
        )
        self.last_reports["gyro"] = report_gyro

        z_hat_accel: List[float] = self._imu_model.predict_accel(self._state)
        nu_accel: List[float] = self._imu_model.residual_accel(
            imu_packet.z_accel,
            z_hat_accel,
        )
        H_accel: List[List[float]] = self._imu_model.jacobian_accel(self._state)
        self._state, self._covariance, report_accel = self._update_step.update(
            self._state,
            self._covariance,
            H_accel,
            imu_packet.R_accel,
            nu_accel,
        )
        self.last_reports["accel"] = report_accel

    def update_mag(self, mag_packet: MagPacket) -> None:
        """Apply magnetometer update."""
        z_hat: List[float] = self._mag_model.predict(self._state)
        nu: List[float] = self._mag_model.residual(mag_packet.z_m, z_hat)
        H: List[List[float]] = self._mag_model.jacobian(self._state)
        self._state, self._covariance, report = self._update_step.update(
            self._state,
            self._covariance,
            H,
            mag_packet.R_m_raw,
            nu,
        )
        self.last_reports["mag"] = report

    def update_stationary(self, stationary_packet: StationaryPacket) -> None:
        """Apply stationary pseudo-updates in deterministic order."""
        if not stationary_packet.is_stationary:
            return
        if self._enable_no_turn and stationary_packet.R_omega is not None:
            z_hat_omega: List[float] = self._stationary_model.predict_no_turn(
                self._state
            )
            nu_omega: List[float] = self._stationary_model.residual_no_turn(
                [0.0, 0.0, 0.0],
                z_hat_omega,
            )
            H_omega: List[List[float]] = self._stationary_model.jacobian_no_turn(
                self._state
            )
            self._state, self._covariance, report = self._update_step.update(
                self._state,
                self._covariance,
                H_omega,
                stationary_packet.R_omega,
                nu_omega,
            )
            self.last_reports["no_turn"] = report

        z_hat_v: List[float] = self._stationary_model.predict_zupt(self._state)
        nu_v: List[float] = self._stationary_model.residual_zupt(
            [0.0, 0.0, 0.0],
            z_hat_v,
        )
        H_v: List[List[float]] = self._stationary_model.jacobian_zupt(self._state)
        self._state, self._covariance, report = self._update_step.update(
            self._state,
            self._covariance,
            H_v,
            stationary_packet.R_v,
            nu_v,
        )
        self.last_reports["zupt"] = report

    def get_state(self) -> AhrsState:
        """Return the current mean state."""
        return self._state

    def get_covariance(self) -> AhrsCovariance:
        """Return the current covariance."""
        return self._covariance

    def _apply_calibration_prior_once(self, imu_packet: ImuPacket) -> None:
        """Apply calibration prior from the first valid IMU packet once."""
        if self._calibration_prior_applied:
            return
        prior_raw: object = imu_packet.calibration_prior
        if not isinstance(prior_raw, Mapping):
            return
        if prior_raw.get("valid") is not True:
            return
        prior: Mapping[str, object] = prior_raw

        applied_any: bool = False
        b_g: List[float] | None = _parse_float_list(
            prior.get("gyro_bias_rads"),
            3,
        )
        b_a: List[float] | None = _parse_float_list(
            prior.get("accel_bias_mps2"),
            3,
        )
        A_a: List[List[float]] | None = _parse_square_row_major(
            prior.get("accel_A_row_major"),
            3,
        )
        if b_g is not None or b_a is not None or A_a is not None:
            applied_any = True
            state: AhrsState = self._state
            self._state = AhrsState(
                p_WB=state.p_WB,
                v_WB=state.v_WB,
                q_WB=state.q_WB,
                omega_WB=state.omega_WB,
                b_g=b_g if b_g is not None else state.b_g,
                b_a=b_a if b_a is not None else state.b_a,
                A_a=A_a if A_a is not None else state.A_a,
                T_BI=state.T_BI,
                T_BM=state.T_BM,
                g_W=state.g_W,
                m_W=state.m_W,
            )

        covariance_updated: bool = False
        P: List[List[float]] = self._covariance.as_matrix()

        gyro_cov: List[List[float]] | None = _parse_square_row_major(
            prior.get("gyro_bias_cov_row_major"),
            3,
        )
        if gyro_cov is not None:
            covariance_updated = _write_cov_block(
                P,
                StateMapping.slice_delta_b_g(),
                gyro_cov,
            )

        accel_cov: List[List[float]] | None = _parse_square_row_major(
            prior.get("accel_param_cov_row_major_12x12"),
            12,
        )
        if accel_cov is not None:
            covariance_updated = (
                _write_accel_param_cov(P, accel_cov) or covariance_updated
            )

        if covariance_updated:
            self._covariance = AhrsCovariance.from_matrix(P)

        applied_any = applied_any or covariance_updated
        if applied_any:
            self._calibration_prior_applied = True


def _build_process_noise(params: AhrsParams) -> List[List[float]]:
    """Return the canonical 39x39 process noise covariance."""
    size: int = 39
    Q_c: List[List[float]] = [[0.0 for _ in range(size)] for _ in range(size)]
    idx: int = 0

    # (m/s^2 / sqrt(s))^2 acceleration smoothness prior variance
    sigma_w_v2: float = params.sigma_w_v * params.sigma_w_v
    _fill_block_diag(Q_c, idx, 3, sigma_w_v2)
    idx += 3

    # (rad/s^2 / sqrt(s))^2 angular acceleration prior variance
    sigma_w_omega2: float = params.sigma_w_omega * params.sigma_w_omega
    _fill_block_diag(Q_c, idx, 3, sigma_w_omega2)
    idx += 3

    # (rad/s / sqrt(s))^2 gyro bias random-walk variance
    sigma_w_bg2: float = params.sigma_w_bg * params.sigma_w_bg
    _fill_block_diag(Q_c, idx, 3, sigma_w_bg2)
    idx += 3

    # (m/s^2 / sqrt(s))^2 accel bias random-walk variance
    sigma_w_ba2: float = params.sigma_w_ba * params.sigma_w_ba
    _fill_block_diag(Q_c, idx, 3, sigma_w_ba2)
    idx += 3

    # (1 / sqrt(s))^2 accel calibration random-walk variance
    sigma_w_Aa2: float = params.sigma_w_Aa * params.sigma_w_Aa
    _fill_block_diag(Q_c, idx, 9, sigma_w_Aa2)
    idx += 9

    # (m / sqrt(s))^2 IMU extrinsics translation random-walk variance
    sigma_w_BI_rho2: float = params.sigma_w_BI_rho * params.sigma_w_BI_rho
    _fill_block_diag(Q_c, idx, 3, sigma_w_BI_rho2)
    idx += 3

    # (rad / sqrt(s))^2 IMU extrinsics rotation random-walk variance
    sigma_w_BI_theta2: float = params.sigma_w_BI_theta * params.sigma_w_BI_theta
    _fill_block_diag(Q_c, idx, 3, sigma_w_BI_theta2)
    idx += 3

    # (m / sqrt(s))^2 mag extrinsics translation random-walk variance
    sigma_w_BM_rho2: float = params.sigma_w_BM_rho * params.sigma_w_BM_rho
    _fill_block_diag(Q_c, idx, 3, sigma_w_BM_rho2)
    idx += 3

    # (rad / sqrt(s))^2 mag extrinsics rotation random-walk variance
    sigma_w_BM_theta2: float = params.sigma_w_BM_theta * params.sigma_w_BM_theta
    _fill_block_diag(Q_c, idx, 3, sigma_w_BM_theta2)
    idx += 3

    # (m/s^2 / sqrt(s))^2 gravity random-walk variance
    sigma_w_g2: float = params.sigma_w_g * params.sigma_w_g
    _fill_block_diag(Q_c, idx, 3, sigma_w_g2)
    idx += 3

    # (tesla / sqrt(s))^2 magnetic field random-walk variance
    sigma_w_m2: float = params.sigma_w_m * params.sigma_w_m
    _fill_block_diag(Q_c, idx, 3, sigma_w_m2)

    return Q_c


def _fill_block_diag(
    mat: List[List[float]],
    start: int,
    size: int,
    value: float,
) -> None:
    """Fill a diagonal block with the provided scalar."""
    i: int
    for i in range(size):
        mat[start + i][start + i] = value


def _identity(size: int) -> List[List[float]]:
    """Return identity matrix."""
    result: List[List[float]] = [[0.0 for _ in range(size)] for _ in range(size)]
    i: int
    for i in range(size):
        result[i][i] = 1.0
    return result


def _copy_matrix(matrix: Sequence[Sequence[float]]) -> List[List[float]]:
    """Return a deep copy of a matrix."""
    return [[float(value) for value in row] for row in matrix]


def _parse_float_list(value: object, length: int) -> List[float] | None:
    """Return a float list if value is a finite list of the right length."""
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        return None
    if len(value) != length:
        return None
    output: List[float] = []
    item: object
    for item in value:
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            return None
        item_f: float = float(item)
        if not math.isfinite(item_f):
            return None
        output.append(item_f)
    return output


def _parse_square_row_major(value: object, size: int) -> List[List[float]] | None:
    """Return a square matrix from a row-major list if valid."""
    values: List[float] | None = _parse_float_list(value, size * size)
    if values is None:
        return None
    return _reshape_row_major(values, size)


def _reshape_row_major(values: Sequence[float], size: int) -> List[List[float]]:
    """Return a square matrix from row-major values."""
    matrix: List[List[float]] = []
    index: int = 0
    row_index: int
    for row_index in range(size):
        row: List[float] = []
        col_index: int
        for col_index in range(size):
            row.append(float(values[index + col_index]))
        matrix.append(row)
        index += size
    return matrix


def _write_cov_block(
    P: List[List[float]],
    block_slice: slice,
    block: Sequence[Sequence[float]],
) -> bool:
    """Write a symmetric covariance block into P."""
    if block_slice.start is None or block_slice.stop is None:
        return False
    size: int = block_slice.stop - block_slice.start
    if len(block) != size or any(len(row) != size for row in block):
        return False
    i: int
    for i in range(size):
        j: int
        for j in range(i, size):
            value: float = 0.5 * (float(block[i][j]) + float(block[j][i]))
            P[block_slice.start + i][block_slice.start + j] = value
            P[block_slice.start + j][block_slice.start + i] = value
    return True


def _write_accel_param_cov(P: List[List[float]], accel_cov: List[List[float]]) -> bool:
    """Write the 12x12 accel bias/A covariance blocks into P."""
    if len(accel_cov) != 12 or any(len(row) != 12 for row in accel_cov):
        return False
    b_a_slice: slice = StateMapping.slice_delta_b_a()
    A_a_slice: slice = StateMapping.slice_delta_A_a()
    if (
        b_a_slice.start is None
        or b_a_slice.stop is None
        or A_a_slice.start is None
        or A_a_slice.stop is None
    ):
        return False
    b_a_start: int = b_a_slice.start
    A_a_start: int = A_a_slice.start

    i: int
    for i in range(3):
        j_bias: int
        for j_bias in range(i, 3):
            value: float = 0.5 * (
                float(accel_cov[i][j_bias]) + float(accel_cov[j_bias][i])
            )
            P[b_a_start + i][b_a_start + j_bias] = value
            P[b_a_start + j_bias][b_a_start + i] = value
    for i in range(9):
        j_A: int
        for j_A in range(i, 9):
            value: float = 0.5 * (
                float(accel_cov[3 + i][3 + j_A])
                + float(accel_cov[3 + j_A][3 + i])
            )
            P[A_a_start + i][A_a_start + j_A] = value
            P[A_a_start + j_A][A_a_start + i] = value
    for i in range(3):
        j_cross: int
        for j_cross in range(9):
            value: float = 0.5 * (
                float(accel_cov[i][3 + j_cross])
                + float(accel_cov[3 + j_cross][i])
            )
            P[b_a_start + i][A_a_start + j_cross] = value
            P[A_a_start + j_cross][b_a_start + i] = value
    return True
