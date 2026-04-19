################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

from __future__ import annotations


def frame_matches(actual_frame_id: str, expected_frame_id: str) -> bool:
    """
    Return True when a source frame matches the configured frame policy.
    """

    return actual_frame_id == expected_frame_id
