################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

from pathlib import Path


def usbc_max_current_ma(sysfs: Path = Path("/sys")) -> int | None:
    """Return the highest configured device current in mA across USB-C ports."""
    limits: list[int] = []
    for port in (sysfs / "bus/usb/devices").glob("*/port"):
        try:
            # The NUC14RVS omits the Type-C connector link at USB location 2
            if not (port / "connector").exists() and not (
                (port / "location").read_text().strip() == "0x80000002"
                and (sysfs / "class/dmi/id/board_name").read_text().strip()
                == "NUC14RVS"
            ):
                continue
            power: str = (port.parent / "bMaxPower").read_text().strip()
            limits.append(int(power.removesuffix("mA")))
        except (OSError, ValueError):
            continue
    return max(limits, default=None)
