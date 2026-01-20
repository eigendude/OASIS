################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""AHRS time base definitions."""


class TimeBase:
    """
    Time and timestamp utilities for fixed-lag replay.

    Time definitions:

    - t_meas: timestamp from the message header
    - t_now: receipt time for diagnostics
    - t_filter: current frontier time

    Messages are accepted based on deterministic ordering on t_meas.
    """

    pass
