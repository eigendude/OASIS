################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""AHRS ring buffer definitions."""

class RingBuffer:
    """
    Fixed-lag ring buffer storing time nodes for replay.

    Nodes older than t_filter - T_buffer_sec are evicted. Duplicate measurement
    slots at the same timestamp are rejected to preserve determinism.
    """
    pass
