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
from types import ModuleType


PACKAGE_ROOT: Path = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT: Path = PACKAGE_ROOT.parent
CONTROL_PACKAGE_ROOT: Path = REPOSITORY_ROOT / "oasis_control"
HOME_PACKAGE_ROOT: Path = REPOSITORY_ROOT / "oasis_home"

for package_root in (PACKAGE_ROOT, CONTROL_PACKAGE_ROOT, HOME_PACKAGE_ROOT):
    package_path: str = str(package_root)
    if package_path not in sys.path:
        sys.path.insert(0, package_path)


from test.ros_test_stubs import install_test_stubs  # noqa: I202


install_test_stubs()


def _install_telemetrix_stub() -> None:
    """Install the subset of Telemetrix needed while importing the bridge."""

    try:
        __import__("telemetrix_aio")
    except ModuleNotFoundError:
        telemetrix_package: ModuleType = ModuleType("telemetrix_aio")
        private_constants_module: ModuleType = ModuleType(
            "telemetrix_aio.private_constants"
        )
        telemetrix_module: ModuleType = ModuleType("telemetrix_aio.telemetrix_aio")

        class PrivateConstants:
            ONEWIRE_FEATURE: int = 0x01
            DHT_FEATURE: int = 0x02
            STEPPERS_FEATURE: int = 0x04
            SPI_FEATURE: int = 0x08
            SERVO_FEATURE: int = 0x10
            SONAR_FEATURE: int = 0x20
            I2C_FEATURE: int = 0x40

        class TelemetrixAIO:
            pass

        setattr(private_constants_module, "PrivateConstants", PrivateConstants)
        setattr(telemetrix_module, "TelemetrixAIO", TelemetrixAIO)
        setattr(telemetrix_package, "private_constants", private_constants_module)
        setattr(telemetrix_package, "telemetrix_aio", telemetrix_module)
        sys.modules["telemetrix_aio"] = telemetrix_package
        sys.modules["telemetrix_aio.private_constants"] = private_constants_module
        sys.modules["telemetrix_aio.telemetrix_aio"] = telemetrix_module


_install_telemetrix_stub()
