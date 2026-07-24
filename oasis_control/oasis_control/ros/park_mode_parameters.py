################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""ROS parameter integration for train park mode."""

import rclpy.node

from oasis_control.input.park_mode import DEFAULT_PARK_MODE_ACCEL_SEC
from oasis_control.input.park_mode import DEFAULT_PARK_MODE_COMMAND
from oasis_control.input.park_mode import DEFAULT_PARK_MODE_ENABLED
from oasis_control.input.park_mode import DEFAULT_PARK_MODE_HOLD_SEC
from oasis_control.input.park_mode import DEFAULT_PARK_MODE_PRELOAD_COMMAND
from oasis_control.input.park_mode import DEFAULT_PARK_MODE_PRELOAD_SEC
from oasis_control.input.park_mode import DEFAULT_PARK_MODE_TAKEUP_COMMAND
from oasis_control.input.park_mode import DEFAULT_PARK_MODE_TAKEUP_SEC
from oasis_control.input.park_mode import ParkModeLaunchProfile
from oasis_control.input.park_mode import validated_park_mode_profile


# ROS parameter name for enabling automatic park mode
PARAM_PARK_MODE_ENABLED: str = "park_mode_enabled"

# ROS parameter name for the fast-preload command
PARAM_PARK_MODE_PRELOAD_COMMAND: str = "park_mode_preload_command"

# ROS parameter name for the fast-preload duration
PARAM_PARK_MODE_PRELOAD_SEC: str = "park_mode_preload_sec"

# ROS parameter name for the magnetic-coupler take-up command
PARAM_PARK_MODE_TAKEUP_COMMAND: str = "park_mode_takeup_command"

# ROS parameter name for the magnetic-coupler take-up duration
PARAM_PARK_MODE_TAKEUP_SEC: str = "park_mode_takeup_sec"

# ROS parameter name for the take-up hold duration
PARAM_PARK_MODE_HOLD_SEC: str = "park_mode_hold_sec"

# ROS parameter name for the final unboosted park command
PARAM_PARK_MODE_COMMAND: str = "park_mode_command"

# ROS parameter name for the final acceleration duration
PARAM_PARK_MODE_ACCEL_SEC: str = "park_mode_accel_sec"


def load_park_mode_parameters(
    node: rclpy.node.Node,
) -> tuple[bool, ParkModeLaunchProfile]:
    """Declare, read, and validate park-mode parameters."""
    node.declare_parameter(PARAM_PARK_MODE_ENABLED, DEFAULT_PARK_MODE_ENABLED)
    node.declare_parameter(
        PARAM_PARK_MODE_PRELOAD_COMMAND,
        DEFAULT_PARK_MODE_PRELOAD_COMMAND,
    )
    node.declare_parameter(
        PARAM_PARK_MODE_PRELOAD_SEC,
        DEFAULT_PARK_MODE_PRELOAD_SEC,
    )
    node.declare_parameter(
        PARAM_PARK_MODE_TAKEUP_COMMAND,
        DEFAULT_PARK_MODE_TAKEUP_COMMAND,
    )
    node.declare_parameter(
        PARAM_PARK_MODE_TAKEUP_SEC,
        DEFAULT_PARK_MODE_TAKEUP_SEC,
    )
    node.declare_parameter(
        PARAM_PARK_MODE_HOLD_SEC,
        DEFAULT_PARK_MODE_HOLD_SEC,
    )
    node.declare_parameter(
        PARAM_PARK_MODE_COMMAND,
        DEFAULT_PARK_MODE_COMMAND,
    )
    node.declare_parameter(
        PARAM_PARK_MODE_ACCEL_SEC,
        DEFAULT_PARK_MODE_ACCEL_SEC,
    )

    enabled: bool = bool(node.get_parameter(PARAM_PARK_MODE_ENABLED).value)
    requested_profile: ParkModeLaunchProfile = ParkModeLaunchProfile(
        preload_command=float(
            node.get_parameter(PARAM_PARK_MODE_PRELOAD_COMMAND).value
        ),
        preload_sec=float(node.get_parameter(PARAM_PARK_MODE_PRELOAD_SEC).value),
        takeup_command=float(node.get_parameter(PARAM_PARK_MODE_TAKEUP_COMMAND).value),
        takeup_sec=float(node.get_parameter(PARAM_PARK_MODE_TAKEUP_SEC).value),
        hold_sec=float(node.get_parameter(PARAM_PARK_MODE_HOLD_SEC).value),
        command=float(node.get_parameter(PARAM_PARK_MODE_COMMAND).value),
        accel_sec=float(node.get_parameter(PARAM_PARK_MODE_ACCEL_SEC).value),
    )
    profile: ParkModeLaunchProfile = validated_park_mode_profile(requested_profile)
    if profile is not requested_profile:
        node.get_logger().warning(
            "Invalid park-mode profile: commands must be finite and in "
            "(0.0, 1.0] with preload < take-up < final, and durations "
            "must be finite and positive except hold, which may be zero; "
            "using complete default profile"
        )

    return enabled, profile
