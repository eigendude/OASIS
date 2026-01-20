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

from typing import Iterable
from typing import List
from typing import Mapping
from typing import Sequence

from oasis_control.localization.ahrs.ahrs_types.imu_packet import ImuPacket
from oasis_control.localization.ahrs.ahrs_types.stationary_packet import (
    StationaryPacket,
)
from oasis_control.localization.ahrs.math_utils.linalg import LinearAlgebra
from oasis_control.localization.ahrs.models.imu_model import ImuModel
from oasis_control.localization.ahrs.state.ahrs_state import AhrsState


class StationaryDetector:
    """Covariance-aware stationary detector for deterministic AHRS replay.

    Responsibility:
        Compute deterministic stationary decisions from buffered IMU windows
        using full covariance whitening metrics and SPD checks.

    Purpose:
        Provide a deterministic mapping from a window of IMU packets and the
        current state to a StationaryPacket with ZUPT/no-turn covariances.

    Inputs/outputs:
        - Inputs: window_imu_packets (sequence of ImuPacket), AhrsState.
        - Outputs: StationaryPacket with window bounds, decision flag, and
          full R_v/R_omega covariances.

    Dependencies:
        - Uses LinearAlgebra for SPD checks and clamping.
        - Uses ImuModel math for residual definitions.
        - Driven by AhrsConfig thresholds and baseline covariances.

    Public API (to be implemented):
        - detect(window_imu_packets, state)

    Data contract:
        - window_imu_packets are sorted deterministically by t_meas_ns.
        - Each packet supplies full R_omega and R_accel (3, 3) covariances.
        - R_v and R_omega outputs are full SPD matrices.
        - t_meas_ns is defined as window_end_ns using exact int nanoseconds.

    Frames and units:
        - Gyro residual ν_ω in {I}, units rad/s.
        - Accel residual ν_a in {I}, units m/s^2.
        - Covariances use squared units and remain full.

    Determinism and edge cases:
        - Identical buffered IMU packets must yield identical decisions.
        - Window bounds are defined by integer-ns min/max timestamps.
        - Reject the window if any R_omega/R_accel is non-SPD.
        - Empty windows must return is_stationary False with metadata.
        - No epsilon or float-time comparisons are allowed.

    Equations:
        Gyro whitening per sample:
            ν_ω,i = z_ω,i - (R_IB * ω_WB + b_g)
            d2_ω,i = ν_ω,iᵀ R_ω^{-1} ν_ω,i

        Accel whitening per sample:
            f_B = R_WB * (0 - g_W)
            f_I = R_IB * f_B
            a_hat_I = A_a^{-1} * f_I + b_a
            ν_a,i = z_a,i - a_hat_I
            d2_a,i = ν_a,iᵀ R_a^{-1} ν_a,i

        Aggregate deterministically:
            D_ω = mean(d2_ω,i)
            D_a = mean(d2_a,i)

        Stationary decision:
            is_stationary = (D_ω < τ_ω) AND (D_a < τ_a)
            plus optional deterministic delta-variance tests.

    Numerical stability notes:
        - Never diagonalize covariances.
        - Clamp R_v and R_omega with LinearAlgebra.clamp_spd.

    Suggested unit tests:
        - Deterministic ordering yields identical D_ω/D_a.
        - Reject window when any covariance is non-SPD.
    """

    @staticmethod
    def detect(
        window_imu_packets: Sequence[ImuPacket],
        state: AhrsState,
        *,
        tau_omega: float,
        tau_accel: float,
        R_v_base: Sequence[Sequence[float]],
        R_omega_base: Sequence[Sequence[float]],
    ) -> StationaryPacket:
        """Return a deterministic stationary decision for the IMU window."""
        if not window_imu_packets:
            metadata_empty: Mapping[str, object] = {
                "reason": "empty window",
                "tau_omega": tau_omega,
                "tau_accel": tau_accel,
                "sample_count": 0,
            }
            return StationaryPacket(
                t_meas_ns=0,
                window_start_ns=0,
                window_end_ns=0,
                is_stationary=False,
                R_v=_clamp_base_covariance(R_v_base),
                R_omega=_clamp_base_covariance(R_omega_base),
                metadata=metadata_empty,
            )

        packets: List[ImuPacket] = sorted(
            window_imu_packets,
            key=lambda packet: packet.t_meas_ns,
        )
        window_start_ns: int = packets[0].t_meas_ns
        window_end_ns: int = packets[-1].t_meas_ns

        if not _all_covariances_spd(packets):
            metadata_reject: Mapping[str, object] = {
                "reason": "non-spd covariance in window",
                "tau_omega": tau_omega,
                "tau_accel": tau_accel,
                "sample_count": len(packets),
            }
            return StationaryPacket(
                t_meas_ns=window_end_ns,
                window_start_ns=window_start_ns,
                window_end_ns=window_end_ns,
                is_stationary=False,
                R_v=_clamp_base_covariance(R_v_base),
                R_omega=_clamp_base_covariance(R_omega_base),
                metadata=metadata_reject,
            )

        d2_omega: List[float] = []
        d2_accel: List[float] = []
        packet: ImuPacket
        for packet in packets:
            z_hat_omega: List[float] = ImuModel.predict_gyro(state)
            nu_omega: List[float] = ImuModel.residual_gyro(packet.z_omega, z_hat_omega)
            d2_omega.append(_whitened_d2(packet.R_omega, nu_omega))

            z_hat_accel: List[float] = ImuModel.predict_accel(state)
            nu_accel: List[float] = ImuModel.residual_accel(packet.z_accel, z_hat_accel)
            d2_accel.append(_whitened_d2(packet.R_accel, nu_accel))

        D_omega: float = _mean(d2_omega)
        D_accel: float = _mean(d2_accel)
        is_stationary: bool = (D_omega < tau_omega) and (D_accel < tau_accel)

        metadata_final: Mapping[str, object] = {
            "D_omega": D_omega,
            "D_accel": D_accel,
            "tau_omega": tau_omega,
            "tau_accel": tau_accel,
            "sample_count": len(packets),
        }
        return StationaryPacket(
            t_meas_ns=window_end_ns,
            window_start_ns=window_start_ns,
            window_end_ns=window_end_ns,
            is_stationary=is_stationary,
            R_v=_clamp_base_covariance(R_v_base),
            R_omega=_clamp_base_covariance(R_omega_base),
            metadata=metadata_final,
        )


def _all_covariances_spd(packets: Iterable[ImuPacket]) -> bool:
    packet: ImuPacket
    for packet in packets:
        if not LinearAlgebra.is_spd(packet.R_omega):
            return False
        if not LinearAlgebra.is_spd(packet.R_accel):
            return False
    return True


def _whitened_d2(R: Sequence[Sequence[float]], nu: Sequence[float]) -> float:
    x: List[float] = LinearAlgebra.solve_spd(R, nu)
    d2: float = 0.0
    for i in range(3):
        d2 += nu[i] * x[i]
    return d2


def _mean(values: Sequence[float]) -> float:
    if not values:
        raise ValueError("values must be non-empty")
    acc: float = 0.0
    for value in values:
        acc += value
    return acc / float(len(values))


def _clamp_base_covariance(
    R_base: Sequence[Sequence[float]],
) -> List[List[float]]:
    min_diag: float = _min_diagonal(R_base)
    max_diag: float = _max_diagonal(R_base)
    min_eig: float = max(min_diag, 1e-12)
    max_eig: float = max(max_diag, min_eig)
    return LinearAlgebra.clamp_spd(R_base, min_eig, max_eig)


def _min_diagonal(R: Sequence[Sequence[float]]) -> float:
    if len(R) != 3 or any(len(row) != 3 for row in R):
        raise ValueError("Covariance must be 3x3")
    return min(R[0][0], R[1][1], R[2][2])


def _max_diagonal(R: Sequence[Sequence[float]]) -> float:
    if len(R) != 3 or any(len(row) != 3 for row in R):
        raise ValueError("Covariance must be 3x3")
    return max(R[0][0], R[1][1], R[2][2])
