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


PACKAGE_ROOT: Path = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT: Path = PACKAGE_ROOT.parent
CONTROL_PACKAGE_ROOT: Path = REPOSITORY_ROOT / "oasis_control"

for package_root in (PACKAGE_ROOT, CONTROL_PACKAGE_ROOT):
    package_path: str = str(package_root)
    if package_path not in sys.path:
        sys.path.insert(0, package_path)


from test.ros_test_stubs import install_test_stubs  # noqa: I202


install_test_stubs()
