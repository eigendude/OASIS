################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""ROS-independent kid-mode train command processing."""


def kid_mode_train_command(left_y: float, right_y: float) -> float:
    """Return a train command from normalized vertical stick coordinates."""
    safe_left_y: float = max(-1.0, min(left_y, 1.0))
    safe_right_y: float = max(-1.0, min(right_y, 1.0))

    forward: float = max(0.0, safe_left_y, safe_right_y)
    reverse: float = max(0.0, -safe_left_y, -safe_right_y)
    return forward - reverse
