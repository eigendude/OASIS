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
    """Tests for the RingBuffer policies."""

    def test_insert_and_get(self) -> None:
        """Inserted nodes can be retrieved by timestamp."""
        buffer: RingBuffer = RingBuffer(100)
        node: TimelineNode = TimelineNode(50)
        inserted: bool = buffer.insert(node, t_filter_ns=50)
        fetched: TimelineNode | None = buffer.get(50)
        self.assertTrue(inserted)
        self.assertIs(fetched, node)

    def test_duplicate_node_rejected(self) -> None:
        """Duplicate node inserts are rejected and counted."""
        buffer: RingBuffer = RingBuffer(100)
        first: TimelineNode = TimelineNode(75)
        second: TimelineNode = TimelineNode(75)
        first_inserted: bool = buffer.insert(first, t_filter_ns=75)
        second_inserted: bool = buffer.insert(second, t_filter_ns=75)
        self.assertTrue(first_inserted)
        self.assertFalse(second_inserted)
        self.assertEqual(buffer.diagnostics["duplicate_node"], 1)

    def test_reject_too_old(self) -> None:
        """Nodes older than the horizon are rejected."""
        buffer: RingBuffer = RingBuffer(50)
        node: TimelineNode = TimelineNode(100)
        inserted: bool = buffer.insert(node, t_filter_ns=200)
        self.assertFalse(inserted)
        self.assertEqual(buffer.diagnostics["reject_too_old"], 1)

    def test_eviction_on_frontier_advance(self) -> None:
        """Nodes older than the horizon are evicted."""
        buffer: RingBuffer = RingBuffer(50)
        first: TimelineNode = TimelineNode(100)
        second: TimelineNode = TimelineNode(200)
        inserted_first: bool = buffer.insert(first, t_filter_ns=100)
        inserted_second: bool = buffer.insert(second, t_filter_ns=200)
        remaining: int = buffer.size()
        self.assertTrue(inserted_first)
        self.assertTrue(inserted_second)
        self.assertEqual(remaining, 1)
        self.assertIsNone(buffer.get(100))
        self.assertEqual(buffer.diagnostics["evicted"], 1)


if __name__ == "__main__":
    unittest.main()
