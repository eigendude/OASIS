################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Test configuration for importing the local Python package under pytest."""

from __future__ import annotations

import sys
from pathlib import Path
from test.ros_test_stubs import install_test_stubs


PACKAGE_ROOT: Path = Path(__file__).resolve().parents[1]
REPO_ROOT: Path = PACKAGE_ROOT.parent
DRIVERS_PACKAGE_ROOT: Path = REPO_ROOT / "oasis_drivers_py"

for package_root in (PACKAGE_ROOT, DRIVERS_PACKAGE_ROOT):
    package_root_str: str = str(package_root)
    if package_root_str not in sys.path:
        sys.path.insert(0, package_root_str)


install_test_stubs()
