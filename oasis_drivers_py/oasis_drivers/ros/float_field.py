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


def to_ros_float(value: float | None) -> float:
    """Encode a missing measurement in a ROS floating-point field."""
    return math.nan if value is None else value
