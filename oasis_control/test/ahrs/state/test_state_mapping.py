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

from oasis_control.localization.ahrs.state.state_mapping import StateMapping


def _slice_size(slc: slice) -> int:
    """Return the size of a slice."""
    if slc.start is None or slc.stop is None:
        raise ValueError("Slice must have start and stop")
    return slc.stop - slc.start


def _collect_slices() -> List[slice]:
    """Return all state mapping slices in order."""
    return [
        StateMapping.slice_delta_p(),
        StateMapping.slice_delta_v(),
        StateMapping.slice_delta_theta(),
        StateMapping.slice_delta_omega(),
        StateMapping.slice_delta_b_g(),
        StateMapping.slice_delta_b_a(),
        StateMapping.slice_delta_A_a(),
        StateMapping.slice_delta_xi_BI(),
        StateMapping.slice_delta_xi_BM(),
        StateMapping.slice_delta_g_W(),
        StateMapping.slice_delta_m_W(),
    ]


class TestStateMapping(unittest.TestCase):
    """Tests for StateMapping layout."""

    def test_dimension(self) -> None:
        """Dimension returns the canonical size."""
        self.assertEqual(StateMapping.dimension(), 45)

    def test_slice_coverage(self) -> None:
        """Slices cover the full range contiguously."""
        slices: List[slice] = _collect_slices()
        expected_start: int = 0
        slc: slice
        for slc in slices:
            self.assertEqual(slc.start, expected_start)
            if slc.stop is None:
                self.fail("Slice stop must be set")
            expected_start = slc.stop
        self.assertEqual(expected_start, 45)

        sizes: List[int] = [_slice_size(slc) for slc in slices]
        expected_sizes: List[int] = [3, 3, 3, 3, 3, 3, 9, 6, 6, 3, 3]
        self.assertEqual(sizes, expected_sizes)


if __name__ == "__main__":
    unittest.main()
