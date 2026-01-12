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

from types import ModuleType

import pytest

np: ModuleType = pytest.importorskip(
    "numpy", reason="TODO: requires numpy for EKF tests"
)

from oasis_control.localization.ekf.core.ekf_core import EkfCore
from oasis_control.localization.ekf.ekf_buffer import EkfBuffer
from oasis_control.localization.ekf.ekf_config import EkfConfig
from oasis_control.localization.ekf.ekf_types import EkfEvent
from oasis_control.localization.ekf.ekf_types import EkfEventType
from oasis_control.localization.ekf.ekf_types import EkfTime
from oasis_control.localization.ekf.ekf_types import MagSample
from oasis_control.localization.ekf.ekf_types import from_ns


_NS_PER_S: int = 1_000_000_000


def _build_config(
    *,
    mag_alpha: float = 0.01,
    mag_r_min: list[float] | None = None,
    mag_r_max: list[float] | None = None,
    mag_r0_default: list[float] | None = None,
    mag_world_t: list[float] | None = None,
) -> EkfConfig:
    mag_r_min_list: list[float] = (
        mag_r_min
        if mag_r_min is not None
        else [
            1.0e-12,
            0.0,
            0.0,
            0.0,
            1.0e-12,
            0.0,
            0.0,
            0.0,
            1.0e-12,
        ]
    )
    mag_r_max_list: list[float] = (
        mag_r_max
        if mag_r_max is not None
        else [
            2.5e-9,
            0.0,
            0.0,
            0.0,
            2.5e-9,
            0.0,
            0.0,
            0.0,
            2.5e-9,
        ]
    )
    mag_r0_list: list[float] = (
        mag_r0_default
        if mag_r0_default is not None
        else [
            4.0e-10,
            0.0,
            0.0,
            0.0,
            4.0e-10,
            0.0,
            0.0,
            0.0,
            4.0e-10,
        ]
    )
    mag_world: list[float] = mag_world_t if mag_world_t is not None else [1.0, 0.0, 0.0]
    return EkfConfig(
        world_frame_id="world",
        odom_frame_id="odom",
        body_frame_id="base_link",
        t_buffer_ns=2 * _NS_PER_S,
        epsilon_wall_future_ns=50_000_000,
        dt_clock_jump_max_ns=_NS_PER_S,
        dt_imu_max_ns=50_000_000,
        pos_var=1.0,
        vel_var=1.0,
        ang_var=1.0,
        accel_noise_var=0.1,
        gyro_noise_var=0.1,
        gravity_mps2=9.81,
        max_dt_ns=50_000_000,
        checkpoint_interval_ns=500_000_000,
        apriltag_pos_var=1.0,
        apriltag_yaw_var=1.0,
        apriltag_gate_d2=0.0,
        apriltag_reproj_rms_gate_px=0.0,
        tag_size_m=0.16,
        tag_anchor_family="tag36h11",
        tag_anchor_id=0,
        tag_landmark_prior_sigma_t_m=0.1,
        tag_landmark_prior_sigma_rot_rad=0.1,
        extrinsic_prior_sigma_t_m=1.0,
        extrinsic_prior_sigma_rot_rad=3.141592653589793,
        mag_alpha=mag_alpha,
        mag_r_min=mag_r_min_list,
        mag_r_max=mag_r_max_list,
        mag_r0_default=mag_r0_list,
        mag_world_t=mag_world,
    )


def _build_mag_sample(*, field_t: list[float], cov: list[float]) -> MagSample:
    return MagSample(
        frame_id="mag",
        magnetic_field_t=field_t,
        magnetic_field_cov=cov,
    )


def test_mag_covariance_initializes_from_first_valid_cov() -> None:
    mag_cov: list[float] = [
        1.0e-9,
        0.0,
        0.0,
        0.0,
        1.0e-9,
        0.0,
        0.0,
        0.0,
        1.0e-9,
    ]
    config: EkfConfig = _build_config(
        mag_r0_default=[
            2.0e-10,
            0.0,
            0.0,
            0.0,
            2.0e-10,
            0.0,
            0.0,
            0.0,
            2.0e-10,
        ]
    )
    core: EkfCore = EkfCore(config)
    event: EkfEvent = EkfEvent(
        t_meas=from_ns(0),
        event_type=EkfEventType.MAG,
        payload=_build_mag_sample(field_t=[0.0, 0.0, 0.0], cov=mag_cov),
    )

    core.process_event(event)

    expected: np.ndarray = np.asarray(mag_cov, dtype=float).reshape(3, 3)
    assert np.allclose(core.mag_covariance(), expected)


def test_mag_covariance_falls_back_to_default() -> None:
    default_cov: list[float] = [
        4.0e-10,
        0.0,
        0.0,
        0.0,
        4.0e-10,
        0.0,
        0.0,
        0.0,
        4.0e-10,
    ]
    config: EkfConfig = _build_config(mag_r0_default=default_cov)
    core: EkfCore = EkfCore(config)
    event: EkfEvent = EkfEvent(
        t_meas=from_ns(0),
        event_type=EkfEventType.MAG,
        payload=_build_mag_sample(field_t=[0.0, 0.0, 0.0], cov=[0.0] * 9),
    )

    core.process_event(event)

    expected: np.ndarray = np.asarray(default_cov, dtype=float).reshape(3, 3)
    assert np.allclose(core.mag_covariance(), expected)


def test_mag_covariance_update_clamps_spd_bounds() -> None:
    mag_r_min: list[float] = [
        1.0e-12,
        0.0,
        0.0,
        0.0,
        1.0e-12,
        0.0,
        0.0,
        0.0,
        1.0e-12,
    ]
    mag_r_max: list[float] = [
        1.0e-9,
        0.0,
        0.0,
        0.0,
        1.0e-9,
        0.0,
        0.0,
        0.0,
        1.0e-9,
    ]
    mag_r0_default: list[float] = [
        1.0e-10,
        0.0,
        0.0,
        0.0,
        1.0e-10,
        0.0,
        0.0,
        0.0,
        1.0e-10,
    ]
    config: EkfConfig = _build_config(
        mag_alpha=0.5,
        mag_r_min=mag_r_min,
        mag_r_max=mag_r_max,
        mag_r0_default=mag_r0_default,
        mag_world_t=[0.0, 0.0, 0.0],
    )
    core: EkfCore = EkfCore(config)
    mag_cov: list[float] = [
        1.0e-10,
        0.0,
        0.0,
        0.0,
        1.0e-10,
        0.0,
        0.0,
        0.0,
        1.0e-10,
    ]
    event: EkfEvent = EkfEvent(
        t_meas=from_ns(0),
        event_type=EkfEventType.MAG,
        payload=_build_mag_sample(field_t=[5.0, 0.0, 0.0], cov=mag_cov),
    )

    core.process_event(event)

    updated: np.ndarray = core.mag_covariance()
    eigenvalues: np.ndarray = np.linalg.eigvalsh(updated)
    min_bound: float = float(np.min(np.linalg.eigvalsh(np.array(mag_r_min))))
    max_bound: float = float(np.max(np.linalg.eigvalsh(np.array(mag_r_max))))
    assert np.all(eigenvalues >= min_bound - 1.0e-18)
    assert np.all(eigenvalues <= max_bound + 1.0e-18)


def test_mag_covariance_replay_determinism() -> None:
    config: EkfConfig = _build_config(mag_world_t=[0.0, 0.0, 0.0])
    event_early: EkfEvent = EkfEvent(
        t_meas=from_ns(_NS_PER_S),
        event_type=EkfEventType.MAG,
        payload=_build_mag_sample(
            field_t=[0.5, 0.0, 0.0],
            cov=[
                1.0e-9,
                0.0,
                0.0,
                0.0,
                1.0e-9,
                0.0,
                0.0,
                0.0,
                1.0e-9,
            ],
        ),
    )
    event_late: EkfEvent = EkfEvent(
        t_meas=from_ns(2 * _NS_PER_S),
        event_type=EkfEventType.MAG,
        payload=_build_mag_sample(
            field_t=[1.5, 0.0, 0.0],
            cov=[
                1.0e-9,
                0.0,
                0.0,
                0.0,
                1.0e-9,
                0.0,
                0.0,
                0.0,
                1.0e-9,
            ],
        ),
    )

    ordered_core: EkfCore = EkfCore(config)
    ordered_core.process_event(event_early)
    ordered_core.process_event(event_late)
    ordered_state: np.ndarray = ordered_core.state()
    ordered_cov: np.ndarray = ordered_core.covariance()
    ordered_mag_cov: np.ndarray = ordered_core.mag_covariance()

    replay_core: EkfCore = EkfCore(config)
    buffer: EkfBuffer = EkfBuffer(config)
    buffer.insert_event(event_late)
    replay_core.process_event(event_late)
    buffer.insert_event(event_early)
    replay_core.replay(buffer, event_early.t_meas)

    assert np.allclose(replay_core.state(), ordered_state)
    assert np.allclose(replay_core.covariance(), ordered_cov)
    assert np.allclose(replay_core.mag_covariance(), ordered_mag_cov)
