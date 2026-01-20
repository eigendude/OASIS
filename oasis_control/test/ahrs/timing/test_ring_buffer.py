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

from oasis_control.localization.ahrs.timing.ring_buffer import RingBuffer
from oasis_control.localization.ahrs.timing.timeline_node import TimelineNode


class TestRingBuffer(unittest.TestCase):
    """Tests for RingBuffer."""

    def test_insert_get_round_trip(self) -> None:
        """Insert returns a retrievable node."""
        buffer: RingBuffer = RingBuffer(t_buffer_ns=100)
        node: TimelineNode = TimelineNode(t_meas_ns=50)
        inserted: bool = buffer.insert(node, t_filter_ns=0)
        self.assertTrue(inserted)
        fetched: TimelineNode | None = buffer.get(50)
        self.assertIs(fetched, node)

    def test_duplicate_node_rejected(self) -> None:
        """Duplicate node insertions are rejected and counted."""
        buffer: RingBuffer = RingBuffer(t_buffer_ns=100)
        node: TimelineNode = TimelineNode(t_meas_ns=50)
        self.assertTrue(buffer.insert(node, t_filter_ns=0))
        self.assertFalse(buffer.insert(node, t_filter_ns=0))
        self.assertEqual(buffer.diagnostics["duplicate_node"], 1)

    def test_reject_too_old(self) -> None:
        """Insert rejects nodes older than the buffer horizon."""
        buffer: RingBuffer = RingBuffer(t_buffer_ns=100)
        node: TimelineNode = TimelineNode(t_meas_ns=50)
        inserted: bool = buffer.insert(node, t_filter_ns=200)
        self.assertFalse(inserted)
        self.assertEqual(buffer.diagnostics["reject_too_old"], 1)

    def test_eviction_on_insert(self) -> None:
        """Insert evicts nodes older than the horizon."""
        buffer: RingBuffer = RingBuffer(t_buffer_ns=100)
        node_old: TimelineNode = TimelineNode(t_meas_ns=50)
        node_new: TimelineNode = TimelineNode(t_meas_ns=150)
        self.assertTrue(buffer.insert(node_old, t_filter_ns=60))
        self.assertTrue(buffer.insert(node_new, t_filter_ns=160))
        self.assertIsNone(buffer.get(50))
        self.assertIs(buffer.get(150), node_new)
        self.assertEqual(buffer.diagnostics["evicted"], 1)
