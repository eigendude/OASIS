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

import json
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

from oasis_control.localization.ahrs.ahrs_types.diagnostics import AhrsDiagnostics


class TestAhrsDiagnostics(unittest.TestCase):
    """Tests for the AhrsDiagnostics container."""

    def _make_diagnostics(self) -> AhrsDiagnostics:
        return AhrsDiagnostics(
            buffer_size=5,
            t_filter_ns=10,
            drop_count=1,
            duplicate_count=2,
            replay_count=3,
            last_replay_from_ns=9,
            rejected_old_count=0,
            rejected_future_count=1,
            duplicate_imu_count=0,
            duplicate_mag_count=0,
            out_of_order_insert_count=1,
            evicted_node_count=2,
        )

    def test_validate_accepts_valid(self) -> None:
        """Validate accepts a properly populated diagnostics payload."""
        diagnostics: AhrsDiagnostics = self._make_diagnostics()
        diagnostics.validate()

    def test_validate_rejects_negative(self) -> None:
        """Validate rejects negative counters."""
        diagnostics: AhrsDiagnostics = AhrsDiagnostics(
            buffer_size=1,
            t_filter_ns=1,
            drop_count=-1,
            duplicate_count=0,
            replay_count=0,
            last_replay_from_ns=0,
            rejected_old_count=0,
            rejected_future_count=0,
            duplicate_imu_count=0,
            duplicate_mag_count=0,
            out_of_order_insert_count=0,
            evicted_node_count=0,
        )
        with self.assertRaises(ValueError):
            diagnostics.validate()

    def test_reset_counters(self) -> None:
        """reset_counters preserves state while zeroing counters."""
        diagnostics: AhrsDiagnostics = self._make_diagnostics()
        reset: AhrsDiagnostics = diagnostics.reset_counters()
        self.assertEqual(reset.buffer_size, diagnostics.buffer_size)
        self.assertEqual(reset.t_filter_ns, diagnostics.t_filter_ns)
        self.assertEqual(reset.last_replay_from_ns, diagnostics.last_replay_from_ns)
        self.assertEqual(reset.drop_count, 0)
        self.assertEqual(reset.replay_count, 0)
        self.assertEqual(reset.evicted_node_count, 0)

    def test_as_dict_json_serializable(self) -> None:
        """as_dict returns JSON-serializable data."""
        diagnostics: AhrsDiagnostics = self._make_diagnostics()
        payload: dict[str, int] = diagnostics.as_dict()
        json.dumps(payload)


if __name__ == "__main__":
    unittest.main()
