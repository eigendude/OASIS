################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""AHRS replay engine definitions."""


class ReplayEngine:
    """
    Deterministic replay engine for out-of-order measurements.

    Insertion of a measurement attaches it to its time node. If inserted in the
    past, the engine replays the process and updates forward to the frontier in a
    stable measurement order.
    """

    pass
