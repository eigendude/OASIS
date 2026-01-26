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
import sys
import unittest
from pathlib import Path
from typing import List


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


def _module_available(module: str) -> bool:
    root: str = module.split(".", maxsplit=1)[0]
    if importlib.util.find_spec(root) is None:
        return False
    return importlib.util.find_spec(module) is not None


REQUIRED_MODULES: list[str] = [
    "builtin_interfaces.msg",
    "geometry_msgs.msg",
    "message_filters",
    "rclpy",
    "sensor_msgs.msg",
    "tf2_ros",
]
missing: list[str] = [
    module for module in REQUIRED_MODULES if not _module_available(module)
]
if missing:
    raise unittest.SkipTest("ROS dependencies are unavailable: " + ", ".join(missing))

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from sensor_msgs.msg import Imu as ImuMsg

from oasis_control.localization.ahrs.ahrs_types.imu_packet import ImuPacket
from oasis_control.localization.ahrs.state.ahrs_state import AhrsState
from oasis_control.localization.ahrs.state.covariance import AhrsCovariance
from oasis_control.localization.ahrs.state.state_mapping import StateMapping
from oasis_control.localization.ahrs.timing.timeline_node import TimelineNode
from oasis_control.nodes.ahrs_node import AhrsNode
from oasis_control.nodes.ahrs_node import _make_imu_fused_msg


class FakeBroadcaster:
    def __init__(self) -> None:
        self.transforms: list[object] = []

    def sendTransform(self, transform: object) -> None:
        self.transforms.append(transform)


class FakePublisher:
    def __init__(self) -> None:
        self.messages: list[ImuMsg] = []

    def publish(self, msg: ImuMsg) -> None:
        self.messages.append(msg)


def _identity_rotation() -> list[list[float]]:
    return [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ]


def _make_state(
    *,
    q_wb: list[float],
    b_g: list[float],
    b_a: list[float],
    t_bi_rotation: list[list[float]],
) -> AhrsState:
    base: AhrsState = AhrsState.reset()
    t_bi: tuple[list[list[float]], list[float]] = (
        [list(row) for row in t_bi_rotation],
        [0.0, 0.0, 0.0],
    )
    return AhrsState(
        p_WB=base.p_WB,
        v_WB=base.v_WB,
        q_WB=q_wb,
        omega_WB=base.omega_WB,
        b_g=b_g,
        b_a=b_a,
        A_a=base.A_a,
        T_BI=t_bi,
        T_BM=base.T_BM,
        g_W=base.g_W,
        m_W=base.m_W,
    )


def _make_imu_packet(
    *,
    z_omega: list[float],
    z_accel: list[float],
    R_omega: list[list[float]],
    R_accel: list[list[float]],
) -> ImuPacket:
    calibration_prior: dict[str, object] = {"valid": True}
    calibration_meta: dict[str, object] = {"source": "test"}
    return ImuPacket(
        t_meas_ns=1_000_000_000,
        frame_id="imu",
        z_omega=z_omega,
        R_omega=R_omega,
        z_accel=z_accel,
        R_accel=R_accel,
        calibration_prior=calibration_prior,
        calibration_meta=calibration_meta,
    )


def _make_covariance(
    *,
    dtheta: list[list[float]],
    dbg: list[list[float]],
    dba: list[list[float]],
) -> AhrsCovariance:
    size: int = StateMapping.dimension()
    P: list[list[float]] = [[0.0 for _ in range(size)] for _ in range(size)]
    _insert_block(P, StateMapping.slice_delta_theta(), dtheta)
    _insert_block(P, StateMapping.slice_delta_b_g(), dbg)
    _insert_block(P, StateMapping.slice_delta_b_a(), dba)
    return AhrsCovariance.from_matrix(P)


def _insert_block(
    matrix: list[list[float]],
    block_slice: slice,
    block: list[list[float]],
) -> None:
    if block_slice.start is None or block_slice.stop is None:
        raise ValueError("block_slice must have explicit bounds")
    start: int = block_slice.start
    stop: int = block_slice.stop
    row_idx: int
    for row_idx in range(start, stop):
        col_idx: int
        for col_idx in range(start, stop):
            matrix[row_idx][col_idx] = block[row_idx - start][col_idx - start]


def _flatten(matrix: list[list[float]]) -> list[float]:
    return [value for row in matrix for value in row]


def _add_matrices(
    left: list[list[float]],
    right: list[list[float]],
) -> list[list[float]]:
    rows: int = len(left)
    summed: list[list[float]] = []
    row_idx: int
    for row_idx in range(rows):
        row: list[float] = []
        col_idx: int
        for col_idx in range(rows):
            row.append(left[row_idx][col_idx] + right[row_idx][col_idx])
        summed.append(row)
    return summed


class TestAhrsImuFusedHelper(unittest.TestCase):
    """Tests for AHRS fused IMU helper"""

    def test_orientation_identity_t_bi(self) -> None:
        """Identity IMU rotation preserves q_WB orientation"""
        stamp: TimeMsg = TimeMsg(sec=1, nanosec=2)
        q_wb: list[float] = [0.5, 0.5, 0.5, 0.5]
        state: AhrsState = _make_state(
            q_wb=q_wb,
            b_g=[0.0, 0.0, 0.0],
            b_a=[0.0, 0.0, 0.0],
            t_bi_rotation=_identity_rotation(),
        )
        covariance: AhrsCovariance = _make_covariance(
            dtheta=_identity_rotation(),
            dbg=_identity_rotation(),
            dba=_identity_rotation(),
        )
        imu_packet: ImuPacket = _make_imu_packet(
            z_omega=[0.0, 0.0, 0.0],
            z_accel=[0.0, 0.0, 0.0],
            R_omega=_identity_rotation(),
            R_accel=_identity_rotation(),
        )

        msg: ImuMsg = _make_imu_fused_msg(
            stamp=stamp,
            frame_id="imu",
            state=state,
            covariance=covariance,
            imu_packet=imu_packet,
        )

        self.assertAlmostEqual(msg.orientation.x, q_wb[1])
        self.assertAlmostEqual(msg.orientation.y, q_wb[2])
        self.assertAlmostEqual(msg.orientation.z, q_wb[3])
        self.assertAlmostEqual(msg.orientation.w, q_wb[0])

    def test_covariance_and_bias_corrections(self) -> None:
        """Covariance blocks and bias corrections are applied"""
        stamp: TimeMsg = TimeMsg(sec=2, nanosec=3)
        state: AhrsState = _make_state(
            q_wb=[1.0, 0.0, 0.0, 0.0],
            b_g=[0.1, 0.2, 0.3],
            b_a=[1.0, 1.5, 2.0],
            t_bi_rotation=_identity_rotation(),
        )
        dtheta: list[list[float]] = [
            [1.0, 0.1, 0.2],
            [0.1, 2.0, 0.3],
            [0.2, 0.3, 3.0],
        ]
        dbg: list[list[float]] = [
            [4.0, 0.4, 0.5],
            [0.4, 5.0, 0.6],
            [0.5, 0.6, 6.0],
        ]
        dba: list[list[float]] = [
            [7.0, 0.7, 0.8],
            [0.7, 8.0, 0.9],
            [0.8, 0.9, 9.0],
        ]
        covariance: AhrsCovariance = _make_covariance(
            dtheta=dtheta,
            dbg=dbg,
            dba=dba,
        )
        R_omega: list[list[float]] = [
            [0.1, 0.0, 0.0],
            [0.0, 0.2, 0.0],
            [0.0, 0.0, 0.3],
        ]
        R_accel: list[list[float]] = [
            [0.4, 0.0, 0.0],
            [0.0, 0.5, 0.0],
            [0.0, 0.0, 0.6],
        ]
        imu_packet: ImuPacket = _make_imu_packet(
            z_omega=[1.1, 2.2, 3.3],
            z_accel=[4.0, 5.5, 7.0],
            R_omega=R_omega,
            R_accel=R_accel,
        )

        msg: ImuMsg = _make_imu_fused_msg(
            stamp=stamp,
            frame_id="imu",
            state=state,
            covariance=covariance,
            imu_packet=imu_packet,
        )

        self.assertAlmostEqual(msg.angular_velocity.x, 1.0)
        self.assertAlmostEqual(msg.angular_velocity.y, 2.0)
        self.assertAlmostEqual(msg.angular_velocity.z, 3.0)
        self.assertAlmostEqual(msg.linear_acceleration.x, 3.0)
        self.assertAlmostEqual(msg.linear_acceleration.y, 4.0)
        self.assertAlmostEqual(msg.linear_acceleration.z, 5.0)

        expected_orientation_cov: list[float] = _flatten(dtheta)
        expected_gyro_cov: list[float] = _flatten(_add_matrices(dbg, R_omega))
        expected_accel_cov: list[float] = _flatten(_add_matrices(dba, R_accel))

        self.assertEqual(len(msg.orientation_covariance), 9)
        self.assertEqual(len(msg.angular_velocity_covariance), 9)
        self.assertEqual(len(msg.linear_acceleration_covariance), 9)
        self.assertEqual(list(msg.orientation_covariance), expected_orientation_cov)
        self.assertEqual(list(msg.angular_velocity_covariance), expected_gyro_cov)
        self.assertEqual(list(msg.linear_acceleration_covariance), expected_accel_cov)


class TestAhrsImuFusedPublish(unittest.TestCase):
    """Tests for fused IMU publishing logic"""

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self) -> None:
        self._node: AhrsNode = AhrsNode(broadcaster=FakeBroadcaster())
        self._fake_pub: FakePublisher = FakePublisher()
        self._node._imu_fused_pub = self._fake_pub

    def tearDown(self) -> None:
        self._node.stop()

    def test_publish_skipped_without_imu_packet(self) -> None:
        """Missing imu_packet skips imu_fused publish"""
        t_filter_ns: int = 1_000_000_000
        timeline_node: TimelineNode = TimelineNode(t_meas_ns=t_filter_ns)
        inserted: bool = self._node._replay_engine.ring_buffer.insert(
            timeline_node,
            t_filter_ns=t_filter_ns,
        )
        self.assertTrue(inserted)

        self._node._publish_frontier(t_filter_ns)

        self.assertEqual(self._fake_pub.messages, [])
