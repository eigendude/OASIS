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
import re
import subprocess
import sys
import time


# Seconds allowed for the firmware read within the 0.5 s telemetry period
_READ_TIMEOUT_S: float = 0.2

# Seconds between inconclusive probes while firmware support is unknown
_PROBE_INTERVAL_S: float = 60.0

# The 5 V input rail has at most two digits before the decimal point
_EXT5V: re.Pattern[bytes] = re.compile(
    rb"^[ \t]*EXT5V_V[ \t]+volt\(\d+\)=(\d{1,2}(?:\.\d+)?)V[ \t]*\r?$", re.MULTILINE
)


class InputVoltageReader:
    """Sample the Raspberry Pi input rail, probing support on the first read."""

    def __init__(self) -> None:
        # None samples every tick after a successful probe. Infinity skips a
        # platform or firmware command known to be unavailable.
        self._probe_after: float | None = 0.0 if sys.platform == "linux" else math.inf

    def read(self) -> float | None:
        """Return the current input voltage in volts, or None if unavailable."""
        if self._probe_after is not None and time.monotonic() < self._probe_after:
            return None

        try:
            result: subprocess.CompletedProcess[bytes] = subprocess.run(
                ["vcgencmd", "pmic_read_adc", "EXT5V_V"],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                check=True,
                timeout=_READ_TIMEOUT_S,
            )
        except (FileNotFoundError, PermissionError):
            self._probe_after = math.inf
            return None
        except (OSError, subprocess.SubprocessError):
            pass
        else:
            output: bytes = result.stdout
            match: re.Match[bytes] | None = _EXT5V.search(output)
            if match is not None:
                self._probe_after = None
                return float(match[1])

            # Older firmware can reject the command or list only other rails
            if b"Command not registered" in output or (
                b" volt(" in output and b"EXT5V_V" not in output
            ):
                self._probe_after = math.inf
                return None

        if self._probe_after is not None:
            self._probe_after = time.monotonic() + _PROBE_INTERVAL_S
        return None
