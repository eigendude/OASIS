################################################################################
#
#  Copyright (C) 2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# Manager for a Millenium Engineer LEGO model
#

from typing import Optional

import rclpy.node
import rclpy.qos
from rclpy.logging import LoggingSeverity
from std_msgs.msg import Header as HeaderMsg

from oasis_control.managers.sampling_manager import SamplingManager
from oasis_control.mcu.mcu_memory_manager import McuMemoryManager
from oasis_msgs.msg import EngineerState as EngineerStateMsg


################################################################################
# Hardware configuration
################################################################################


# Sampling interval, in ms
SAMPLING_INTERVAL_MS: int = 100

# Memory reporting interval, in seconds
MEMORY_INTERVAL_SECS: float = 10.0  # RAM utilization doesn't currently change


################################################################################
# ROS parameters
################################################################################


ROS_NAMESPACE = "oasis"

NODE_NAME = "engineer_manager"

PUBLISH_STATE_PERIOD_SECS = 0.1

# Publisher
PUBLISH_ENGINEER_STATE = "engineer_state"


################################################################################
# ROS node
################################################################################


class EngineerManagerNode(rclpy.node.Node):
    """
    A ROS node that manages a Millenium Engineer LEGO model.
    """

    def __init__(self) -> None:
        """
        Initialize resources.
        """
        super().__init__(NODE_NAME)

        # Enable debug logging
        self.get_logger().set_level(LoggingSeverity.DEBUG)

        # Subsystems
        self._mcu_memory_manager: McuMemoryManager = McuMemoryManager(self)
        self._sampling_manager: SamplingManager = SamplingManager(self)

        # Reliable listener QOS profile for publishers
        qos_profile: rclpy.qos.QoSPresetProfile = (
            rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value
        )

        # Publishers
        self._engineer_state_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=EngineerStateMsg,
            topic=PUBLISH_ENGINEER_STATE,
            qos_profile=qos_profile,
        )

        # Timer parameters
        self._publish_state_timer: Optional[rclpy.node.Timer] = None

    def initialize(self) -> bool:
        self.get_logger().debug("Starting engineer manager configuration")

        # Memory reporting
        if not self._mcu_memory_manager.initialize(MEMORY_INTERVAL_SECS):
            return False

        # Sampling interval
        if not self._sampling_manager.initialize(SAMPLING_INTERVAL_MS):
            return False

        # Now that the manager is initialized, start the publishing timer
        self._publish_state_timer = self.create_timer(
            timer_period_sec=PUBLISH_STATE_PERIOD_SECS, callback=self._publish_state
        )

        self.get_logger().info("Engineer manager initialized successfully")

        return True

    def _publish_state(self) -> None:
        header = HeaderMsg()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = NODE_NAME  # TODO

        msg: EngineerStateMsg = EngineerStateMsg()
        msg.header = header
        msg.total_ram = self._mcu_memory_manager.total_ram
        msg.ram_utilization = self._mcu_memory_manager.ram_utilization

        self._engineer_state_pub.publish(msg)
