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

from oasis_control.localization.ahrs.timing.time_base import TimeBase


class TestTimeBase(unittest.TestCase):
    """Tests for TimeBase helpers."""

    def test_validate_non_negative(self) -> None:
        """validate_non_negative accepts non-negative ints."""
        TimeBase.validate_non_negative(0)
        TimeBase.validate_non_negative(10)
        with self.assertRaisesRegex(ValueError, "t_ns must be non-negative"):
            TimeBase.validate_non_negative(-1)
        with self.assertRaisesRegex(ValueError, "t_ns must be non-negative"):
            TimeBase.validate_non_negative(True)

    def test_validate_monotonic(self) -> None:
        """validate_monotonic rejects decreasing timestamps."""
        TimeBase.validate_monotonic(5, 5)
        TimeBase.validate_monotonic(5, 6)
        with self.assertRaisesRegex(ValueError, "timestamps must be monotonic"):
            TimeBase.validate_monotonic(6, 5)
        with self.assertRaisesRegex(ValueError, "t_ns must be non-negative"):
            TimeBase.validate_monotonic(-1, 5)

    def test_stamp_to_ns(self) -> None:
        """stamp_to_ns converts and validates inputs."""
        result: int = TimeBase.stamp_to_ns(1, 2)
        self.assertEqual(result, 1_000_000_002)
        with self.assertRaisesRegex(ValueError, "sec must be int"):
            TimeBase.stamp_to_ns(True, 0)
        with self.assertRaisesRegex(ValueError, "sec must be non-negative"):
            TimeBase.stamp_to_ns(-1, 0)
        with self.assertRaisesRegex(ValueError, "nanosec must be int"):
            TimeBase.stamp_to_ns(0, False)
        with self.assertRaisesRegex(
            ValueError,
            r"nanosec must be in \[0, 1_000_000_000\)",
        ):
            TimeBase.stamp_to_ns(0, -1)
        with self.assertRaisesRegex(
            ValueError,
            r"nanosec must be in \[0, 1_000_000_000\)",
        ):
            TimeBase.stamp_to_ns(0, 1_000_000_000)
