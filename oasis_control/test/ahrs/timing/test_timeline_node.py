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
from typing import cast

import pytest

from oasis_control.localization.ahrs.ahrs_types.imu_packet import ImuPacket
from oasis_control.localization.ahrs.ahrs_types.mag_packet import MagPacket
from oasis_control.localization.ahrs.ahrs_types.stationary_packet import (
    StationaryPacket,
)
from oasis_control.localization.ahrs.state.ahrs_state import AhrsState
from oasis_control.localization.ahrs.state.covariance import AhrsCovariance
from oasis_control.localization.ahrs.state.state_mapping import StateMapping
from oasis_control.localization.ahrs.timing.timeline_node import TimelineNode


@dataclass(frozen=True, slots=True)
class FakePacket:
    t_meas_ns: int


def _identity_matrix(size: int) -> list[list[float]]:
    return [[1.0 if row == col else 0.0 for col in range(size)] for row in range(size)]


def test_insert_imu_rejects_duplicate() -> None:
    node: TimelineNode = TimelineNode(t_meas_ns=10)
    packet: ImuPacket = cast(ImuPacket, FakePacket(t_meas_ns=10))

    assert node.insert_imu(packet)
    assert not node.insert_imu(packet)
    assert node.diagnostics["duplicate_imu"] == 1


def test_insert_rejects_mismatched_timestamp() -> None:
    node: TimelineNode = TimelineNode(t_meas_ns=10)
    packet: MagPacket = cast(MagPacket, FakePacket(t_meas_ns=11))

    with pytest.raises(ValueError, match="t_meas_ns mismatch"):
        node.insert_mag(packet)


def test_insert_allows_multiple_types() -> None:
    node: TimelineNode = TimelineNode(t_meas_ns=5)
    imu_packet: ImuPacket = cast(ImuPacket, FakePacket(t_meas_ns=5))
    mag_packet: MagPacket = cast(MagPacket, FakePacket(t_meas_ns=5))

    assert node.insert_imu(imu_packet)
    assert node.insert_mag(mag_packet)


def test_is_complete_requires_state_and_covariance() -> None:
    node: TimelineNode = TimelineNode(t_meas_ns=20)
    assert not node.is_complete()

    state: AhrsState = AhrsState.reset()
    cov_size: int = StateMapping.dimension()
    covariance: AhrsCovariance = AhrsCovariance.from_matrix(_identity_matrix(cov_size))

    node.state = state
    node.covariance = covariance
    assert node.is_complete()


def test_insert_stationary_rejects_duplicate() -> None:
    node: TimelineNode = TimelineNode(t_meas_ns=42)
    packet: StationaryPacket = cast(StationaryPacket, FakePacket(t_meas_ns=42))

    assert node.insert_stationary(packet)
    assert not node.insert_stationary(packet)
    assert node.diagnostics["duplicate_stationary"] == 1
