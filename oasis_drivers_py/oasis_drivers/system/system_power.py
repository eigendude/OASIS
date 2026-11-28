################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

import math
import os
import time
from collections.abc import Callable
from pathlib import Path
from typing import BinaryIO


# Seconds between discovery attempts when powercap is absent or inaccessible
_PROBE_INTERVAL_S: float = 60.0

# Microjoules in one joule, for converting energy per second to watts
_UJ_PER_J: int = 1_000_000


def _counter(raw: bytes) -> int:
    raw = raw.strip()
    if not raw.isdigit():
        raise ValueError("Expected an unsigned decimal energy value")
    value: int = int(raw)
    # Linux powercap exports energy and range as unsigned 64-bit values
    if value.bit_length() > 64:
        raise ValueError("Energy exceeds the powercap counter width")
    return value


def _read_energy(path: Path) -> tuple[int, tuple[int, int]]:
    stream: BinaryIO
    with path.open("rb") as stream:
        status: os.stat_result = os.fstat(stream.fileno())
        return _counter(stream.read()), (status.st_dev, status.st_ino)


class SystemPowerReader:
    """Measure average platform power from the Intel RAPL psys energy counter."""

    def __init__(
        self,
        root: Path = Path("/sys/class/powercap"),
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._root: Path = root
        self._clock: Callable[[], float] = clock
        self._domain: tuple[Path, int, tuple[int, int]] | None = None
        self._probe_after: float = -math.inf
        self._previous: tuple[int, float] | None = None

    def _sample(self) -> tuple[int, int] | None:
        path: Path
        maximum: int
        energy: int
        identity: tuple[int, int]
        if self._domain is not None:
            cached_identity: tuple[int, int]
            path, maximum, cached_identity = self._domain
            energy, identity = _read_energy(path)
            if identity == cached_identity:
                return energy, maximum

            # A reloaded driver can reuse a zone index for a different domain
            self._domain = None
            self._previous = None
            self._probe_after = -math.inf

        now: float = self._clock()
        if not math.isfinite(now) or now < self._probe_after:
            return None
        self._probe_after = now + _PROBE_INTERVAL_S

        zone: Path
        for zone in sorted(self._root.glob("intel-rapl*:*")):
            try:
                if (zone / "name").read_bytes().strip() != b"psys":
                    continue
                maximum = _counter((zone / "max_energy_range_uj").read_bytes())
                path = zone / "energy_uj"
                energy, identity = _read_energy(path)
                if 0 < maximum and energy <= maximum:
                    self._domain = (path, maximum, identity)
                    return energy, maximum
            except (OSError, ValueError):
                continue
        return None

    def read(self) -> float | None:
        """Return average platform watts since the previous sample, or None."""
        try:
            sample: tuple[int, int] | None = self._sample()
            if sample is None:
                return None
            energy: int
            maximum: int
            energy, maximum = sample
            timestamp: float = self._clock()
            if energy > maximum or not math.isfinite(timestamp):
                raise ValueError("Invalid platform power sample")
        except (FileNotFoundError, PermissionError):
            self._previous = None
            self._domain = None
            self._probe_after = self._clock() + _PROBE_INTERVAL_S
            return None
        except (OSError, ValueError):
            self._previous = None
            return None

        previous: tuple[int, float] | None = self._previous
        self._previous = (energy, timestamp)
        if previous is None:
            return None

        elapsed: float = timestamp - previous[1]
        if elapsed <= 0.0 or not math.isfinite(elapsed):
            self._previous = None
            return None

        delta_uj: int = energy - previous[0]
        if delta_uj < 0:
            # At rollover, add the energy from the previous count to the limit
            delta_uj += maximum

        return (delta_uj / _UJ_PER_J) / elapsed
