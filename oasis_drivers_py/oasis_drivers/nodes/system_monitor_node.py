#!/usr/bin/env python3
################################################################################
#
#  Copyright (C) 2021 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

import rclpy.node
import rclpy.publisher
import rclpy.qos
from builtin_interfaces.msg import Time as TimeMsg

from oasis_drivers.system.system_monitor import SystemMonitor
from oasis_msgs.msg import SystemTelemetry as SystemTelemetryMsg


################################################################################
# ROS parameters
################################################################################


NODE_NAME = "system_monitor"

PUBLISH_TELEMETRY_TOPIC = "system_telemetry"

READ_PERIOD_SECS = 1.0


################################################################################
# ROS node
################################################################################


class SystemMonitorNode(rclpy.node.Node):
    def __init__(self) -> None:
        """
        Initialize resources.
        """
        super().__init__(NODE_NAME)

        # Reliable listener QOS profile for subscribers
        qos_profile: rclpy.qos.QoSProfile = (
            rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value
        )

        # Publishers
        self._telemetry_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=SystemTelemetryMsg,
            topic=PUBLISH_TELEMETRY_TOPIC,
            qos_profile=qos_profile,
        )

        # Timing parameters
        self._timer: rclpy.node.Timer = self.create_timer(
            timer_period_sec=READ_PERIOD_SECS,
            callback=self._read_psutil,
        )

        self.get_logger().info("System monitor initialized")

    def stop(self) -> None:
        self.get_logger().info("System monitor deinitialized")

        # Destroy the node explicitly. Problems can occur when the garbage
        # collector automatically destroys the node object after ROS has
        # shut down.
        self.destroy_node()

    def _read_psutil(self) -> None:
        timestamp: TimeMsg = self.get_clock().now().to_msg()
        frame_id: str = NODE_NAME  # TODO
        SystemMonitor.read_psutil(self._telemetry_pub, timestamp, frame_id)
