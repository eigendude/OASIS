################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Shared bounded QoS profiles for OASIS control ROS interfaces."""

from __future__ import annotations

import rclpy.qos


MEASUREMENT_QOS_DEPTH: int = 50
STATE_QOS_DEPTH: int = 1


def reliable_measurement_qos() -> rclpy.qos.QoSProfile:
    """Return reliable, volatile, bounded measurement-stream QoS."""

    return rclpy.qos.QoSProfile(
        history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
        depth=MEASUREMENT_QOS_DEPTH,
        reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
        durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
    )


def reliable_state_qos() -> rclpy.qos.QoSProfile:
    """Return reliable, transient-local QoS for a latest-value state."""

    return rclpy.qos.QoSProfile(
        history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
        depth=STATE_QOS_DEPTH,
        reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
        durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )
