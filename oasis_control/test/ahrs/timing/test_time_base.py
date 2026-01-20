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
from typing import cast


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
    """Tests for TimeBase validation and conversion."""

    def test_validate_non_negative(self) -> None:
        """validate_non_negative rejects negative or non-int values."""
        TimeBase.validate_non_negative(0)
        TimeBase.validate_non_negative(5)
        with self.assertRaisesRegex(ValueError, "t_ns must be non-negative"):
            TimeBase.validate_non_negative(-1)
        with self.assertRaisesRegex(ValueError, "t_ns must be non-negative"):
            TimeBase.validate_non_negative(True)

    def test_validate_monotonic(self) -> None:
        """validate_monotonic rejects decreasing timestamps."""
        TimeBase.validate_monotonic(1, 1)
        TimeBase.validate_monotonic(1, 2)
        with self.assertRaisesRegex(ValueError, "timestamps must be monotonic"):
            TimeBase.validate_monotonic(5, 4)

    def test_stamp_to_ns_success(self) -> None:
        """stamp_to_ns returns deterministic nanosecond timestamps."""
        value: int = TimeBase.stamp_to_ns(2, 3)
        self.assertEqual(value, 2_000_000_003)

    def test_stamp_to_ns_invalid(self) -> None:
        """stamp_to_ns rejects invalid sec or nanosec values."""
        with self.assertRaisesRegex(ValueError, "sec must be int"):
            TimeBase.stamp_to_ns(cast(int, 1.0), 0)
        with self.assertRaisesRegex(ValueError, "sec must be non-negative"):
            TimeBase.stamp_to_ns(-1, 0)
        with self.assertRaisesRegex(ValueError, "nanosec must be int"):
            TimeBase.stamp_to_ns(0, cast(int, 1.0))
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


if __name__ == "__main__":
    unittest.main()
