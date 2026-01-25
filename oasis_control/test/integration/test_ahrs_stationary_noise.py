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

import importlib.util
import math
import random
import sys
import unittest
from pathlib import Path
from typing import Any
from typing import Dict
from typing import List
from typing import Mapping
from typing import MutableMapping
from typing import Sequence
from typing import Tuple


ROOT: Path = Path(__file__).resolve().parents[3]
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
from oasis_control.localization.ahrs.models.imu_model import ImuModel
from oasis_control.localization.ahrs.models.mag_model import MagModel
from oasis_control.localization.ahrs.state.ahrs_state import AhrsState
from oasis_control.localization.ahrs.state.covariance import AhrsCovariance
from oasis_control.localization.ahrs.state.state_mapping import StateMapping
from oasis_control.localization.ahrs.timing.replay_engine import ReplayEngine


def _identity(size: int) -> List[List[float]]:
    matrix: List[List[float]] = []
    row_idx: int
    for row_idx in range(size):
        row: List[float] = [0.0 for _ in range(size)]
        row[row_idx] = 1.0
        matrix.append(row)
    return matrix


def _reshape_row_major(values: Sequence[float], size: int) -> List[List[float]]:
    if len(values) != size * size:
        raise ValueError("Unexpected row-major size")
    matrix: List[List[float]] = []
    row_idx: int
    for row_idx in range(size):
        start: int = row_idx * size
        matrix.append([float(value) for value in values[start : start + size]])
    return matrix


def _extract_block(
    matrix: Sequence[Sequence[float]],
    row_slice: slice,
    col_slice: slice,
) -> List[List[float]]:
    rows: List[List[float]] = []
    row: Sequence[float]
    for row in matrix[row_slice]:
        rows.append([float(value) for value in row[col_slice]])
    return rows


def _add_diag_epsilon(
    matrix: Sequence[Sequence[float]], eps: float
) -> List[List[float]]:
    updated: List[List[float]] = []
    row_idx: int
    row: Sequence[float]
    for row_idx, row in enumerate(matrix):
        new_row: List[float] = [float(value) for value in row]
        new_row[row_idx] += eps
        updated.append(new_row)
    return updated


def _diag_stddevs(matrix: Sequence[Sequence[float]], eps: float) -> List[float]:
    stddevs: List[float] = []
    idx: int
    for idx in range(3):
        variance: float = float(matrix[idx][idx])
        stddevs.append(math.sqrt(max(variance, eps)))
    return stddevs


def _is_symmetric(matrix: Sequence[Sequence[float]], tol: float) -> bool:
    size: int = len(matrix)
    i: int
    for i in range(size):
        j: int
        for j in range(i + 1, size):
            if abs(matrix[i][j] - matrix[j][i]) > tol:
                return False
    return True


def _symmetrize_matrix(matrix: Sequence[Sequence[float]]) -> List[List[float]]:
    size: int = len(matrix)
    sym: List[List[float]] = [[0.0 for _ in range(size)] for _ in range(size)]
    i: int
    for i in range(size):
        j: int
        for j in range(size):
            sym[i][j] = 0.5 * (float(matrix[i][j]) + float(matrix[j][i]))
    return sym


def _max_abs_diff(
    lhs: Sequence[Sequence[float]],
    rhs: Sequence[Sequence[float]],
) -> float:
    max_diff: float = 0.0
    row_idx: int
    for row_idx, row in enumerate(lhs):
        col_idx: int
        for col_idx, value in enumerate(row):
            diff: float = abs(float(value) - float(rhs[row_idx][col_idx]))
            if diff > max_diff:
                max_diff = diff
    return max_diff


def _sum_abs(matrix: Sequence[Sequence[float]]) -> float:
    total: float = 0.0
    row: Sequence[float]
    for row in matrix:
        value: float
        for value in row:
            total += abs(float(value))
    return total


def _vector_norm(values: Sequence[float]) -> float:
    return math.sqrt(sum(value * value for value in values))


def _load_yaml(path: Path) -> Mapping[str, Any]:
    yaml_spec = importlib.util.find_spec("yaml")
    if yaml_spec is not None:
        import yaml  # type: ignore[import-not-found]

        with path.open("r", encoding="utf-8") as handle:
            data: object = yaml.safe_load(handle)
        if not isinstance(data, Mapping):
            raise ValueError("YAML root must be a mapping")
        return data
    return _parse_simple_yaml(path.read_text(encoding="utf-8"))


def _parse_simple_yaml(text: str) -> Mapping[str, Any]:
    root: Dict[str, Any] = {}
    stack: List[Tuple[int, MutableMapping[str, Any] | List[Any], str | None]] = [
        (0, root, None)
    ]
    for raw_line in text.splitlines():
        line: str = raw_line.split("#", 1)[0].rstrip()
        if not line.strip():
            continue
        indent: int = len(line) - len(line.lstrip(" "))
        content: str = line.strip()
        if indent == 0:
            stack = stack[:1]
        while stack and indent < stack[-1][0]:
            stack.pop()
        if not stack:
            raise ValueError("Invalid indentation in YAML")
        parent_indent, container, pending_key = stack[-1]
        if indent > parent_indent and pending_key is not None:
            new_container: MutableMapping[str, Any] | List[Any]
            if content.startswith("- "):
                new_container = []
            else:
                new_container = {}
            if isinstance(container, MutableMapping):
                container[pending_key] = new_container
            else:
                raise ValueError("Unexpected list container for mapping")
            stack[-1] = (parent_indent, container, None)
            stack.append((indent, new_container, None))
            container = new_container
        if content.startswith("- "):
            item_str: str = content[2:].strip()
            if isinstance(container, MutableMapping):
                if pending_key is None:
                    raise ValueError("List item without pending key")
                new_list: List[Any] = []
                container[pending_key] = new_list
                stack[-1] = (parent_indent, container, None)
                stack.append((indent, new_list, None))
                container = new_list
            if not isinstance(container, list):
                raise ValueError("Expected list container")
            container.append(_parse_scalar(item_str))
            continue
        if ":" not in content:
            raise ValueError("Expected key-value mapping")
        key, value = content.split(":", 1)
        key = key.strip()
        value_str: str = value.strip()
        if not isinstance(container, MutableMapping):
            raise ValueError("Expected mapping container")
        if value_str == "":
            stack[-1] = (parent_indent, container, key)
            continue
        container[key] = _parse_scalar(value_str)
        stack[-1] = (parent_indent, container, None)
    return root


def _parse_scalar(value: str) -> Any:
    if value in ("true", "True"):
        return True
    if value in ("false", "False"):
        return False
    try:
        if "." in value or "e" in value or "E" in value:
            return float(value)
        return int(value)
    except ValueError:
        return value


class EkfAdapter:
    """Adapter to align AhrsEkf with ReplayEngine's EKF protocol."""

    def __init__(self, ekf: AhrsEkf) -> None:
        self._ekf: AhrsEkf = ekf
        self._last_t_ns: int = 0

    def propagate_to(self, t_ns: int) -> None:
        dt_ns: int = t_ns - self._last_t_ns
        if dt_ns < 0:
            dt_ns = 0
        if dt_ns > 0:
            self._ekf.propagate_to(t_ns)
        self._last_t_ns = t_ns

    def update_imu(self, imu_packet: ImuPacket) -> None:
        self._ekf.update_imu(imu_packet)

    def update_mag(self, mag_packet: MagPacket) -> None:
        self._ekf.update_mag(mag_packet)

    def update_stationary(self, stationary_packet: StationaryPacket) -> None:
        self._ekf.update_stationary(stationary_packet)

    def get_state(self) -> AhrsState:
        return self._ekf.get_state()

    def get_covariance(self) -> AhrsCovariance:
        return self._ekf.get_covariance()


class TestAhrsStationaryNoise(unittest.TestCase):
    """Integration-ish test for stationary IMU+mag with calibration prior."""

    def test_stationary_noise_with_calibration_prior(self) -> None:
        imu_path: Path = (
            Path(__file__).resolve().parents[2]
            / "config"
            / "imu_info"
            / "test_imu_mpu6050_calibration.yaml"
        )
        mag_path: Path = (
            Path(__file__).resolve().parents[2]
            / "config"
            / "magnetometer_info"
            / "test_mag_mmc5983ma_calibration.yaml"
        )
        imu_yaml: Mapping[str, Any] = _load_yaml(imu_path)
        mag_yaml: Mapping[str, Any] = _load_yaml(mag_path)

        gravity: float = float(imu_yaml["gravity_mps2"])
        calib: Mapping[str, Any] = imu_yaml["calib"]
        measurement_noise: Mapping[str, Any] = imu_yaml["measurement_noise"]

        gyro_cov_row_major: Sequence[float]
        if "gyro_cov_corrected_rads2_2" in measurement_noise:
            gyro_cov_row_major = measurement_noise["gyro_cov_corrected_rads2_2"]
        else:
            gyro_cov_row_major = measurement_noise["gyro_cov_raw_rads2_2"]

        accel_cov_row_major: Sequence[float]
        if "accel_cov_corrected_mps2_2" in measurement_noise:
            accel_cov_row_major = measurement_noise["accel_cov_corrected_mps2_2"]
        else:
            accel_cov_row_major = measurement_noise["accel_cov_raw_mps2_2"]

        gyro_cov: List[List[float]] = _reshape_row_major(gyro_cov_row_major, 3)
        accel_cov: List[List[float]] = _reshape_row_major(accel_cov_row_major, 3)
        mag_cov: List[List[float]] = _reshape_row_major(
            mag_yaml["covariance_t2"],
            3,
        )

        eps: float = 1e-18
        R_omega: List[List[float]] = _add_diag_epsilon(gyro_cov, eps)
        R_accel: List[List[float]] = _add_diag_epsilon(accel_cov, eps)
        R_mag: List[List[float]] = _add_diag_epsilon(mag_cov, eps)

        gyro_stddevs: List[float] = _diag_stddevs(R_omega, eps)
        accel_stddevs: List[float] = _diag_stddevs(R_accel, eps)
        mag_stddevs: List[float] = _diag_stddevs(R_mag, eps)

        accel_A_row_major: Sequence[float] = calib["accel_A_row_major"]
        accel_A: List[List[float]] = _reshape_row_major(accel_A_row_major, 3)

        calibration_prior: Dict[str, Any] = {
            "valid": bool(calib["valid"]),
            "gyro_bias_rads": list(calib["gyro_bias_rads"]),
            "gyro_bias_cov_row_major": list(calib["gyro_bias_cov_row_major"]),
            "accel_bias_mps2": list(calib["accel_bias_mps2"]),
            "accel_A_row_major": list(calib["accel_A_row_major"]),
            "accel_param_cov_row_major_12x12": list(
                calib["accel_param_cov_row_major_12x12"]
            ),
        }
        if "temperature_c" in calib:
            calibration_prior["temperature_c"] = float(calib["temperature_c"])
        if "temperature_var_c2" in calib:
            calibration_prior["temperature_var_c2"] = float(calib["temperature_var_c2"])

        offset_t: List[float] = [float(value) for value in mag_yaml["offset_t"]]

        truth_state: AhrsState = AhrsState(
            p_WB=[0.0, 0.0, 0.0],
            v_WB=[0.0, 0.0, 0.0],
            q_WB=[1.0, 0.0, 0.0, 0.0],
            omega_WB=[0.0, 0.0, 0.0],
            b_g=list(calib["gyro_bias_rads"]),
            b_a=list(calib["accel_bias_mps2"]),
            A_a=accel_A,
            T_BI=(_identity(3), [0.0, 0.0, 0.0]),
            T_BM=(_identity(3), [0.0, 0.0, 0.0]),
            g_W=[0.0, 0.0, -gravity],
            m_W=[25e-6, 0.0, 40e-6],
        )

        base_state: AhrsState = AhrsState.reset()
        initial_state: AhrsState = AhrsState(
            p_WB=base_state.p_WB,
            v_WB=base_state.v_WB,
            q_WB=base_state.q_WB,
            omega_WB=base_state.omega_WB,
            b_g=base_state.b_g,
            b_a=base_state.b_a,
            A_a=base_state.A_a,
            T_BI=base_state.T_BI,
            T_BM=base_state.T_BM,
            g_W=[0.0, 0.0, -gravity],
            m_W=truth_state.m_W,
        )

        ekf: AhrsEkf = AhrsEkf(state=initial_state)
        adapter: EkfAdapter = EkfAdapter(ekf)
        engine: ReplayEngine = ReplayEngine(t_buffer_ns=2_000_000_000, ekf=adapter)

        imu_model: ImuModel = ImuModel()
        mag_model: MagModel = MagModel()

        rng: random.Random = random.Random(0)
        dt_ns: int = 20_000_000
        duration_s: float = 2.0
        num_samples: int = int(duration_s / 0.02)
        t0_ns: int = 1_000_000_000
        warmup_samples: int = 5

        initial_cov: List[List[float]] = ekf.get_covariance().as_matrix()
        first_state: AhrsState | None = None
        first_cov: List[List[float]] | None = None
        saw_gyro: bool = False
        saw_accel: bool = False
        saw_mag: bool = False
        saw_zupt: bool = False

        sample_idx: int
        for sample_idx in range(num_samples):
            jitter_ns: int = 0
            if sample_idx % 10 == 0:
                jitter_ns = 1_000_000
            elif sample_idx % 10 == 5:
                jitter_ns = -1_000_000
            t_ns: int = t0_ns + sample_idx * dt_ns + jitter_ns

            noise_scale: float = 0.0 if sample_idx == 0 else 1.0
            gyro_pred: List[float] = imu_model.predict_gyro(truth_state)
            accel_pred: List[float] = imu_model.predict_accel(truth_state)

            z_omega: List[float] = [
                gyro_pred[0] + rng.gauss(0.0, gyro_stddevs[0]) * noise_scale,
                gyro_pred[1] + rng.gauss(0.0, gyro_stddevs[1]) * noise_scale,
                gyro_pred[2] + rng.gauss(0.0, gyro_stddevs[2]) * noise_scale,
            ]
            z_accel: List[float] = [
                accel_pred[0] + rng.gauss(0.0, accel_stddevs[0]) * noise_scale,
                accel_pred[1] + rng.gauss(0.0, accel_stddevs[1]) * noise_scale,
                accel_pred[2] + rng.gauss(0.0, accel_stddevs[2]) * noise_scale,
            ]

            imu_packet: ImuPacket = ImuPacket(
                t_meas_ns=t_ns,
                frame_id="imu",
                z_omega=z_omega,
                R_omega=R_omega,
                z_accel=z_accel,
                R_accel=R_accel,
                calibration_prior=calibration_prior,
                calibration_meta={"source": "stationary-noise"},
            )
            self.assertTrue(engine.insert_imu(imu_packet))

            if sample_idx == 0:
                first_state = ekf.get_state()
                first_cov = ekf.get_covariance().as_matrix()

                for idx, value in enumerate(calib["gyro_bias_rads"]):
                    self.assertAlmostEqual(
                        first_state.b_g[idx],
                        float(value),
                        delta=1e-6,
                    )
                for idx, value in enumerate(calib["accel_bias_mps2"]):
                    self.assertAlmostEqual(
                        first_state.b_a[idx],
                        float(value),
                        delta=1e-6,
                    )

                expected_A_a: List[List[float]] = _reshape_row_major(
                    calib["accel_A_row_major"],
                    3,
                )
                for row_idx, row in enumerate(expected_A_a):
                    for col_idx, value in enumerate(row):
                        self.assertAlmostEqual(
                            first_state.A_a[row_idx][col_idx],
                            float(value),
                            delta=1e-6,
                        )

                b_g_slice: slice = StateMapping.slice_delta_b_g()
                b_a_slice: slice = StateMapping.slice_delta_b_a()
                A_a_slice: slice = StateMapping.slice_delta_A_a()

                gyro_cov_expected_raw: List[List[float]] = _reshape_row_major(
                    calib["gyro_bias_cov_row_major"],
                    3,
                )
                gyro_cov_expected: List[List[float]] = _symmetrize_matrix(
                    gyro_cov_expected_raw
                )
                accel_cov_expected_raw: List[List[float]] = _reshape_row_major(
                    calib["accel_param_cov_row_major_12x12"],
                    12,
                )
                accel_cov_expected: List[List[float]] = _symmetrize_matrix(
                    accel_cov_expected_raw
                )

                b_g_block: List[List[float]] = _extract_block(
                    first_cov,
                    b_g_slice,
                    b_g_slice,
                )
                initial_b_g: List[List[float]] = _extract_block(
                    initial_cov,
                    b_g_slice,
                    b_g_slice,
                )
                near_zero_tol: float = 1e-5
                self.assertTrue(_is_symmetric(b_g_block, tol=1e-9))
                after_b_g_diff: float = _max_abs_diff(
                    b_g_block,
                    gyro_cov_expected,
                )
                initial_b_g_diff: float = _max_abs_diff(
                    initial_b_g,
                    gyro_cov_expected,
                )
                self.assertLess(after_b_g_diff, 1e-6)
                if initial_b_g_diff > near_zero_tol:
                    self.assertLess(after_b_g_diff, initial_b_g_diff)
                else:
                    self.assertLessEqual(after_b_g_diff, near_zero_tol)
                self.assertGreater(_sum_abs(b_g_block), 0.0)

                b_a_block: List[List[float]] = _extract_block(
                    first_cov,
                    b_a_slice,
                    b_a_slice,
                )
                expected_b_a: List[List[float]] = _extract_block(
                    accel_cov_expected,
                    slice(0, 3),
                    slice(0, 3),
                )
                initial_b_a: List[List[float]] = _extract_block(
                    initial_cov,
                    b_a_slice,
                    b_a_slice,
                )
                self.assertTrue(_is_symmetric(b_a_block, tol=1e-9))
                after_b_a_diff: float = _max_abs_diff(
                    b_a_block,
                    expected_b_a,
                )
                initial_b_a_diff: float = _max_abs_diff(
                    initial_b_a,
                    expected_b_a,
                )
                self.assertLess(after_b_a_diff, 1e-3)
                if initial_b_a_diff > near_zero_tol:
                    self.assertLess(after_b_a_diff, initial_b_a_diff)
                else:
                    self.assertLessEqual(after_b_a_diff, near_zero_tol)
                self.assertGreater(_sum_abs(b_a_block), 0.0)

                A_a_block: List[List[float]] = _extract_block(
                    first_cov,
                    A_a_slice,
                    A_a_slice,
                )
                expected_A_a_cov: List[List[float]] = _extract_block(
                    accel_cov_expected,
                    slice(3, 12),
                    slice(3, 12),
                )
                initial_A_a: List[List[float]] = _extract_block(
                    initial_cov,
                    A_a_slice,
                    A_a_slice,
                )
                self.assertTrue(_is_symmetric(A_a_block, tol=1e-9))
                after_A_a_diff: float = _max_abs_diff(
                    A_a_block,
                    expected_A_a_cov,
                )
                initial_A_a_diff: float = _max_abs_diff(
                    initial_A_a,
                    expected_A_a_cov,
                )
                self.assertLess(after_A_a_diff, 1e-3)
                if initial_A_a_diff > near_zero_tol:
                    self.assertLess(after_A_a_diff, initial_A_a_diff)
                else:
                    self.assertLessEqual(after_A_a_diff, near_zero_tol)
                self.assertGreater(_sum_abs(A_a_block), 0.0)

                cross_block: List[List[float]] = _extract_block(
                    first_cov,
                    b_a_slice,
                    A_a_slice,
                )
                expected_cross: List[List[float]] = _extract_block(
                    accel_cov_expected,
                    slice(0, 3),
                    slice(3, 12),
                )
                self.assertLess(
                    _max_abs_diff(cross_block, expected_cross),
                    1e-3,
                )

                self.assertIn("gyro", ekf.last_reports)
                self.assertIn("accel", ekf.last_reports)
                self.assertTrue(ekf.last_reports["gyro"].accepted)
                self.assertTrue(ekf.last_reports["accel"].accepted)
                saw_gyro = True
                saw_accel = True

            if sample_idx % 5 == 0 and sample_idx >= warmup_samples:
                mag_pred: List[float] = mag_model.predict(truth_state)
                z_mag: List[float] = [
                    mag_pred[0]
                    + rng.gauss(0.0, mag_stddevs[0]) * noise_scale
                    - offset_t[0],
                    mag_pred[1]
                    + rng.gauss(0.0, mag_stddevs[1]) * noise_scale
                    - offset_t[1],
                    mag_pred[2]
                    + rng.gauss(0.0, mag_stddevs[2]) * noise_scale
                    - offset_t[2],
                ]
                mag_packet: MagPacket = MagPacket(
                    t_meas_ns=t_ns,
                    frame_id="mag",
                    z_m=z_mag,
                    R_m_raw=R_mag,
                )
                self.assertTrue(engine.insert_mag(mag_packet))
                if "mag" in ekf.last_reports:
                    saw_mag = saw_mag or ekf.last_reports["mag"].accepted

            if sample_idx >= warmup_samples:
                R_v: List[List[float]] = [
                    [0.01, 0.0, 0.0],
                    [0.0, 0.01, 0.0],
                    [0.0, 0.0, 0.01],
                ]
                stationary_packet: StationaryPacket = StationaryPacket(
                    t_meas_ns=t_ns,
                    window_start_ns=t_ns - dt_ns,
                    window_end_ns=t_ns,
                    is_stationary=True,
                    R_v=R_v,
                    R_omega=R_omega,
                    metadata={"source": "stationary-noise"},
                )
                self.assertTrue(engine.insert_stationary(stationary_packet))

                if sample_idx == warmup_samples:
                    self.assertIn("zupt", ekf.last_reports)
                    self.assertTrue(ekf.last_reports["zupt"].accepted)
                    if "no_turn" in ekf.last_reports:
                        self.assertTrue(ekf.last_reports["no_turn"].accepted)
                if "zupt" in ekf.last_reports:
                    saw_zupt = saw_zupt or ekf.last_reports["zupt"].accepted

        self.assertIsNotNone(first_state)
        self.assertIsNotNone(first_cov)
        self.assertTrue(saw_gyro)
        self.assertTrue(saw_accel)
        self.assertTrue(saw_mag)
        self.assertTrue(saw_zupt)

        final_state: AhrsState = ekf.get_state()
        final_cov: List[List[float]] = ekf.get_covariance().as_matrix()

        self.assertTrue(_is_symmetric(final_cov, tol=1e-9))
        cov_row: Sequence[float]
        for cov_row in final_cov:
            cov_value: float
            for cov_value in cov_row:
                self.assertTrue(math.isfinite(cov_value))
        diag_idx: int
        for diag_idx in range(len(final_cov)):
            self.assertGreaterEqual(final_cov[diag_idx][diag_idx], -1e-12)

        self.assertLess(_vector_norm(final_state.v_WB), 0.5)
        self.assertLess(_vector_norm(final_state.b_g), 1.0)
        self.assertAlmostEqual(_vector_norm(final_state.q_WB), 1.0, delta=1e-3)


if __name__ == "__main__":
    unittest.main()
