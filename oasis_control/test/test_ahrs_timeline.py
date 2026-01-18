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

from oasis_control.localization.ahrs.ahrs_timeline import AhrsTimeBuffer
from oasis_control.localization.ahrs.ahrs_timeline import AhrsTimeNode
from oasis_control.localization.ahrs.ahrs_types import AhrsEvent
from oasis_control.localization.ahrs.ahrs_types import AhrsEventType
from oasis_control.localization.ahrs.ahrs_types import AhrsTime
from oasis_control.localization.ahrs.ahrs_types import MagSample


def _event_at(sec: int, nanosec: int) -> AhrsEvent:
    sample: MagSample = MagSample(
        frame_id="mag",
        magnetic_field_t=[0.0, 0.0, 0.0],
        magnetic_field_cov=[0.0] * 9,
    )

    return AhrsEvent(
        t_meas=AhrsTime(sec=sec, nanosec=nanosec),
        topic="magnetic_field",
        frame_id="mag",
        event_type=AhrsEventType.MAG,
        payload=sample,
    )


def test_time_buffer_orders_nodes() -> None:
    buffer: AhrsTimeBuffer = AhrsTimeBuffer()

    buffer.insert_event(_event_at(2, 0))
    buffer.insert_event(_event_at(1, 0))
    buffer.insert_event(_event_at(1, 500))

    nodes: list[AhrsTimeNode] = list(buffer.iter_nodes())
    times: list[tuple[int, int]] = [(node.t.sec, node.t.nanosec) for node in nodes]

    assert times == [(1, 0), (1, 500), (2, 0)]


def test_time_buffer_span_sec() -> None:
    buffer: AhrsTimeBuffer = AhrsTimeBuffer()

    buffer.insert_event(_event_at(1, 0))
    buffer.insert_event(_event_at(3, 0))

    span_sec: float = buffer.span_sec()

    assert span_sec == 2.0
