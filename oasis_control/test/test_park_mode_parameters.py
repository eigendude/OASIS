################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for park-mode ROS parameters."""

from __future__ import annotations

import rclpy.node

from oasis_control.input.park_mode import DEFAULT_PARK_MODE_ENABLED
from oasis_control.input.park_mode import DEFAULT_PARK_MODE_PROFILE
from oasis_control.input.park_mode import ParkModeLaunchProfile
from oasis_control.ros.park_mode_parameters import load_park_mode_parameters


def test_default_park_mode_parameters_load_complete_profile() -> None:
    node: rclpy.node.Node = rclpy.node.Node("park_mode_parameters_test")

    enabled: bool
    profile: ParkModeLaunchProfile
    enabled, profile = load_park_mode_parameters(node)

    assert enabled is DEFAULT_PARK_MODE_ENABLED
    assert profile == DEFAULT_PARK_MODE_PROFILE
