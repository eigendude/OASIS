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

import sys
import unittest
from pathlib import Path
from typing import List
from typing import Sequence


ROOT: Path = Path(__file__).resolve().parents[4]
PACKAGE_ROOT: Path = ROOT / "oasis_control"
PACKAGE_SRC: Path = PACKAGE_ROOT / "oasis_control"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))
if "oasis_control" in sys.modules:
    module_path_raw = getattr(sys.modules["oasis_control"], "__path__", None)
    module_paths: List[str] = []
    if module_path_raw is not None:
        module_paths = list(module_path_raw)
        if str(PACKAGE_SRC) not in module_paths:
            module_paths.append(str(PACKAGE_SRC))
            sys.modules["oasis_control"].__path__ = module_paths

from oasis_control.localization.ahrs.ahrs_types.imu_packet import ImuPacket
from oasis_control.localization.ahrs.ahrs_types.mag_packet import MagPacket
from oasis_control.localization.ahrs.ahrs_types.stationary_packet import (
    StationaryPacket,
)
from oasis_control.localization.ahrs.filter.ekf import AhrsEkf
from oasis_control.localization.ahrs.filter.update_step import UpdateReport
from oasis_control.localization.ahrs.state.ahrs_state import AhrsState
from oasis_control.localization.ahrs.state.covariance import AhrsCovariance
from oasis_control.localization.ahrs.state.state_mapping import StateMapping


def _identity(size: int) -> List[List[float]]:
    """Return an identity matrix."""
    result: List[List[float]] = [[0.0 for _ in range(size)] for _ in range(size)]
    idx: int
    for idx in range(size):
        result[idx][idx] = 1.0
    return result


def _zero_matrix(size: int) -> List[List[float]]:
    """Return a square zero matrix."""
    return [[0.0 for _ in range(size)] for _ in range(size)]


def _make_H(tag: float) -> List[List[float]]:
    """Return a 3xN Jacobian with a tag in the first entry."""
    size: int = StateMapping.dimension()
    H: List[List[float]] = [[0.0 for _ in range(size)] for _ in range(3)]
    H[0][0] = tag
    return H


class FakeImuModel:
    """Fake IMU model for order testing."""

    @staticmethod
    def predict_gyro(state: AhrsState) -> List[float]:
        _ = state
        return [0.0, 0.0, 0.0]

    @staticmethod
    def residual_gyro(z_omega: Sequence[float], z_hat: Sequence[float]) -> List[float]:
        return [z_omega[0] - z_hat[0], z_omega[1] - z_hat[1], z_omega[2] - z_hat[2]]

    @staticmethod
    def jacobian_gyro(state: AhrsState) -> List[List[float]]:
        _ = state
        return _make_H(1.0)

    @staticmethod
    def predict_accel(state: AhrsState) -> List[float]:
        _ = state
        return [0.0, 0.0, 0.0]

    @staticmethod
    def residual_accel(z_accel: Sequence[float], z_hat: Sequence[float]) -> List[float]:
        return [z_accel[0] - z_hat[0], z_accel[1] - z_hat[1], z_accel[2] - z_hat[2]]

    @staticmethod
    def jacobian_accel(state: AhrsState) -> List[List[float]]:
        _ = state
        return _make_H(2.0)


class FakeMagModel:
    """Fake magnetometer model for order testing."""

    @staticmethod
    def predict(state: AhrsState) -> List[float]:
        _ = state
        return [0.0, 0.0, 0.0]

    @staticmethod
    def residual(z_m: Sequence[float], z_hat: Sequence[float]) -> List[float]:
        return [z_m[0] - z_hat[0], z_m[1] - z_hat[1], z_m[2] - z_hat[2]]

    @staticmethod
    def jacobian(state: AhrsState) -> List[List[float]]:
        _ = state
        return _make_H(3.0)


class FakeStationaryModel:
    """Fake stationary model for order testing."""

    @staticmethod
    def predict_no_turn(state: AhrsState) -> List[float]:
        _ = state
        return [0.0, 0.0, 0.0]

    @staticmethod
    def residual_no_turn(
        z_omega: Sequence[float],
        z_hat: Sequence[float],
    ) -> List[float]:
        return [
            z_omega[0] - z_hat[0],
            z_omega[1] - z_hat[1],
            z_omega[2] - z_hat[2],
        ]

    @staticmethod
    def jacobian_no_turn(state: AhrsState) -> List[List[float]]:
        _ = state
        return _make_H(4.0)

    @staticmethod
    def predict_zupt(state: AhrsState) -> List[float]:
        _ = state
        return [0.0, 0.0, 0.0]

    @staticmethod
    def residual_zupt(z_v: Sequence[float], z_hat: Sequence[float]) -> List[float]:
        return [z_v[0] - z_hat[0], z_v[1] - z_hat[1], z_v[2] - z_hat[2]]

    @staticmethod
    def jacobian_zupt(state: AhrsState) -> List[List[float]]:
        _ = state
        return _make_H(5.0)


class FakeUpdateStep:
    """Update-step stub that records the update ordering."""

    def __init__(self, order: List[str]) -> None:
        self._order: List[str] = order

    def update(
        self,
        state: AhrsState,
        covariance: AhrsCovariance,
        H: Sequence[Sequence[float]],
        R: Sequence[Sequence[float]],
        nu: Sequence[float],
    ) -> tuple[AhrsState, AhrsCovariance, UpdateReport]:
        _ = R
        _ = nu
        tag: float = H[0][0]
        if tag == 1.0:
            self._order.append("gyro")
        elif tag == 2.0:
            self._order.append("accel")
        elif tag == 3.0:
            self._order.append("mag")
        elif tag == 4.0:
            self._order.append("no_turn")
        elif tag == 5.0:
            self._order.append("zupt")
        report: UpdateReport = UpdateReport(
            accepted=True,
            reason="",
            innovation_mahalanobis2=0.0,
        )
        return (state, covariance, report)


class TestAhrsEkf(unittest.TestCase):
    """Tests for AhrsEkf."""

    def test_update_ordering(self) -> None:
        """Updates follow the gyro->accel->mag->no-turn->zupt order."""
        order: List[str] = []
        ekf: AhrsEkf = AhrsEkf(
            state=AhrsState.reset(),
            covariance=AhrsCovariance.from_matrix(_identity(StateMapping.dimension())),
            Q_c=_zero_matrix(39),
            imu_model=FakeImuModel(),
            mag_model=FakeMagModel(),
            stationary_model=FakeStationaryModel(),
            update_step=FakeUpdateStep(order),
        )

        imu_packet: ImuPacket = ImuPacket(
            t_meas_ns=0,
            frame_id="imu",
            z_omega=[0.0, 0.0, 0.0],
            R_omega=_identity(3),
            z_accel=[0.0, 0.0, 0.0],
            R_accel=_identity(3),
            calibration_prior={},
            calibration_meta={},
        )
        mag_packet: MagPacket = MagPacket(
            t_meas_ns=0,
            frame_id="mag",
            z_m=[0.0, 0.0, 0.0],
            R_m_raw=_identity(3),
        )
        stationary_packet: StationaryPacket = StationaryPacket(
            t_meas_ns=0,
            window_start_ns=0,
            window_end_ns=0,
            is_stationary=True,
            R_v=_identity(3),
            R_omega=_identity(3),
            metadata={},
        )

        ekf.update_imu(imu_packet)
        ekf.update_mag(mag_packet)
        ekf.update_stationary(stationary_packet)

        self.assertEqual(order, ["gyro", "accel", "mag", "no_turn", "zupt"])

    def test_propagate_monotonic(self) -> None:
        """propagate_to enforces monotonic timestamps."""
        ekf: AhrsEkf = AhrsEkf(Q_c=_zero_matrix(39))
        ekf.propagate_to(10)
        with self.assertRaises(ValueError):
            ekf.propagate_to(5)

    def test_stationary_gate(self) -> None:
        """Stationary updates are skipped when is_stationary is False."""
        order: List[str] = []
        ekf: AhrsEkf = AhrsEkf(
            Q_c=_zero_matrix(39),
            stationary_model=FakeStationaryModel(),
            update_step=FakeUpdateStep(order),
        )
        packet: StationaryPacket = StationaryPacket(
            t_meas_ns=0,
            window_start_ns=0,
            window_end_ns=0,
            is_stationary=False,
            R_v=_identity(3),
            R_omega=_identity(3),
            metadata={},
        )

        ekf.update_stationary(packet)

        self.assertEqual(order, [])


if __name__ == "__main__":
    unittest.main()
