################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""AHRS timeline definitions."""

class Timeline:
    """
    Ordered time line of AHRS nodes for fixed-lag replay.

    Each node is keyed by a unique timestamp and stores the mean state,
    covariance, and attached measurements for deterministic reprocessing.
    """
    pass
