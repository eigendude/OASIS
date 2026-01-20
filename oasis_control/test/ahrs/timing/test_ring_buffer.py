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

from oasis_control.localization.ahrs.timing.ring_buffer import RingBuffer
from oasis_control.localization.ahrs.timing.timeline_node import TimelineNode


def test_insert_and_get_node() -> None:
    ring_buffer: RingBuffer = RingBuffer(t_buffer_ns=10)
    node: TimelineNode = TimelineNode(t_meas_ns=5)

    assert ring_buffer.insert(node, t_filter_ns=5)
    assert ring_buffer.get(5) is node


def test_duplicate_node_rejected() -> None:
    ring_buffer: RingBuffer = RingBuffer(t_buffer_ns=10)
    node: TimelineNode = TimelineNode(t_meas_ns=5)

    assert ring_buffer.insert(node, t_filter_ns=5)
    assert not ring_buffer.insert(node, t_filter_ns=5)
    assert ring_buffer.diagnostics["duplicate_node"] == 1


def test_too_old_node_rejected() -> None:
    ring_buffer: RingBuffer = RingBuffer(t_buffer_ns=10)
    node: TimelineNode = TimelineNode(t_meas_ns=80)

    assert not ring_buffer.insert(node, t_filter_ns=100)
    assert ring_buffer.diagnostics["reject_too_old"] == 1


def test_eviction_occurs_when_frontier_advances() -> None:
    ring_buffer: RingBuffer = RingBuffer(t_buffer_ns=10)
    first: TimelineNode = TimelineNode(t_meas_ns=100)
    second: TimelineNode = TimelineNode(t_meas_ns=120)

    assert ring_buffer.insert(first, t_filter_ns=100)
    assert ring_buffer.insert(second, t_filter_ns=120)

    assert ring_buffer.get(100) is None
    assert ring_buffer.diagnostics["evicted"] == 1
